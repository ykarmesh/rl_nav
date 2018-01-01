from numpy import *

from sklearn.linear_model import SGDRegressor, LinearRegression, Ridge, LogisticRegression
from sklearn.tree import DecisionTreeRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.ensemble import RandomForestRegressor
from sklearn.svm import SVR

from sklearn.mixture import GMM
from cvxopt import matrix
from cvxopt import solvers
import cPickle
import roslaunch
import rospkg

from os.path import expanduser, isfile
import os

class irlAgent:
	def __init__(self): #constructor
		
#		self.expertPolicyFE = [7.94833258, 4.00615625, 3.94217633, 1.70083069, 6.13039255, 5.83950167, 0.25298775, 3.74988035, 2.81007646, 0.30131893, 0.2260963, 0.47487342, 0.22796305]
							   	
							   
		self.expertPolicyFE = [4.00615625, 3.94217633, 1.70083069, 6.13039255, 5.83950167, 
								#0.25298775, 0.47487342, 
								0.22796305]
		self.epsilon = 0.1
		self.policyFEList = [[3.18936675, 2.66387114, 2.72253439, 4.6488438, 3.87395053, 
								#0.16248797, 0.24998246, 
								0.46075135]]
		if isfile('features.txt'):
			featFile = open('features.txt','r')
			policyFEdata = featFile.read().splitlines()
			for i in policyFEdata:
				self.policyFEList.append(map(float,i.split()))
						 #[3.78231822, 4.41765222, 0.28649939, 6.84205623, 6.6908953, 0.27505168, 0.28665525, 0.20000328]]
						 # [9.8094388, 4.71923125, 5.09020754, 0.28028794, 8.44822424, 7.8934127, 0.39055288, 0.27346009, 0.02117347],
						 # [2.2844062, 1.2085572, 1.075849, 2.02644136, 1.41425835, 0.67071795, 0.06215316, 0.32066539, 0.8572882]]

		self.bestPolicy = self.policyFEList[0]
		self.currentT = inf
		self.minimumT = inf

		self.iter = 0
		self.ros_path = expanduser("~")+'/.ros/'

		rospack = rospkg.RosPack()
		self.rl_nav = rospack.get_path('rl_nav')
		self.vslam_path = rospack.get_path('ORB_SLAM2')

		self.GAMMA = 0.9
		self.ALPHA = 0.5

	def phi(self, stateAction):
		temp=[int(stateAction[0]), int(stateAction[1]), int(stateAction[2]), int(stateAction[-1])]
		# temp = []
		# if stateAction[0]:
		# 	temp.append(1)
		# 	temp.append(0)
		# else:
		# 	temp.append(0)
		# 	temp.append(1)

		# temp.append(stateAction[1]/19.0) #angle
		# temp.append(stateAction[2]/19.0) #visible points
		# temp.append(stateAction[3]/19.0) #common points
		# temp.append(stateAction[-3]) #initial covariance
		# temp.append(stateAction[-2]) #final covariance
		# temp.append(stateAction[-1]) # vslam breakage
		# #temp = temp + stateAction[-3:] 
		return tuple(temp)
	
	def QTrainer(self,W):
		W /= linalg.norm(W)
		dataFile = open("qTrain.txt",'r')
		
		data = dataFile.read().splitlines()
		dataFile.close()
		Q = DecisionTreeRegressor()
		# Q.warm_start=True
		# Q.coef_ = array([random.rand()]*len(W[:-2]))
		# Q.intercept_= array([random.rand()])
		print 'TRAINING Q MATRIX'
		# prev_coef = Q.coef_.tolist()
		# prev_inter = Q.intercept_.tolist()
		X = []
		for i in range(len(data)):
				data[i] = data[i].split(';')[:-1]

				data[i][0] = self.phi(map(float,data[i][0].split('\t')[:-1]))
				X.append(data[i][0][:-2])
				for k in range(1,len(data[i])):
					data[i][k] = self.phi(map(float,data[i][k].split('\t')[:-1])+[0])[:-2]
		
		for it in range(10):
			Y = []
			for i in range(len(data)-1):
				
				reward = dot(W,data[i][0])
				sa = data[i][0]
				if it != 0:

					
					if sa[-1]:
						qNext = 0
					else:
						#qNext = max([Q.predict([saNext])[0] for saNext in data[i][1:]])
						qNext = Q.predict([data[i+1][0][:-2]])[0]
					# q = Q.predict([sa[:-1]])[0]
					# targetQ = q + self.ALPHA * (reward + self.GAMMA * qNext - q)
					targetQ = reward + self.GAMMA * qNext
				else:
					targetQ = reward
				Y.append(targetQ)

			Q.fit(X[:len(Y)],Y)
			# print Q.coef_, Q.intercept_
			# if it != 0:
			# 	print it, linalg.norm(array(Q.coef_.tolist() + Q.intercept_.tolist()) - array(prev_coef + prev_inter))
			print it
			# prev_coef = Q.coef_.tolist()
			# prev_inter = Q.intercept_.tolist()
		cPickle.dump(Q,open(self.rl_nav+'/qRegressor.pkl','wb'))
		print 'Q MATRIX TRAINING DONE'
		# Q.intercept_.tolist() + 
		# print "Q_weights",Q.coef_, Q.intercept_
		# if isfile(self.rl_nav+'/qData.txt'):
		# 	os.remove(self.rl_nav+'/qData.txt')
		# qFile = open(self.rl_nav+'/qData.txt', 'w')
		# qFile.write(' '.join(map(str,Q.coef_.tolist() + Q.intercept_.tolist())))
		# qFile.write('\n')
		# qFile.close()
		return Q

	def roslauncher(self, launch_files):
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
		launch.start()
		launch.spin()
		launch.shutdown()

	def getRLAgentFE(self, W): #get the feature expectations of a new poliicy using RL agent
		#self.initQTrainer(W) # offline training of Q-matrix
		self.QTrainer(W)

		#save the weights in a file
		os.remove(self.rl_nav+'/wData.txt')
		wFile = open(self.rl_nav+'/wData.txt', 'w')
		wFile.write('\t'.join(map(str,W)))
		wFile.write('\n')
		wFile.close()

		# # print 'RUNNING ON GAZEBO' # train the Q-matrix further, online
		# # self.roslauncher([self.rl_nav+'/launch/turtlebot_ptam_gazebo.launch', 
		# # 	 			  self.ptam_path+'/launch/ptam.launch', 
		# # 	 			  self.rl_nav+'/launch/turtlebot_joystick_train.launch'])
		
		print 'TESTING ON GAZEBO' #test/run on Gazebo to get the feature expectations
		self.roslauncher([self.rl_nav+'/launch/turtlebot_gazebo.launch', 
			 			  self.vslam_path+'/launch/orb_monocular.launch', 
			 			  self.rl_nav+'/launch/test.launch'])
		
		while not isfile(self.ros_path + 'feFile.txt'):
			print 'TESTING ON GAZEBO' #test/run on Gazebo to get the feature expectations
			self.roslauncher([self.rl_nav+'/launch/turtlebot_gazebo.launch', 
				 			  self.vslam_path+'/launch/orb_monocular.launch', 
			 				  self.rl_nav+'/launch/test.launch'])

		feFile = open(self.ros_path +'feFile.txt', 'r')
		features = feFile.read().splitlines()
		for i in range(len(features)):
			features[i] = map(float, features[i].split('\t')[:-1])
			#features[i] = self.phi(features[i]).tolist()

		features = array(features)
		phi_list = []
		len_episodes = []
		length = 0
		phi = zeros(len(features[0]))
		gamma = 1
		for i in range(len(features)):
			phi = phi + features[i] * gamma
			gamma = gamma * self.GAMMA
			length = length + 1
			if features[i][-1]==1:
				phi_list.append(phi.tolist())
				len_episodes.append(length)
				length = 0
				gamma = 1
				phi = zeros(len(features[0]))
		print "AVERAGE STEPS TO BREAKAGE: ", average(len_episodes)
		feFile.close()
		os.rename(self.ros_path+'feFile.txt','feFile_'+str(len(self.policyFEList))+'.txt')
		return average(phi_list,axis=0)	
		
	def policyListUpdater(self, W):  #add the policyFE list and differences
		lastFE = self.getRLAgentFE(W)
		
		feFile = open('features.txt', 'a')
		feFile.write(' '.join(map(str,lastFE)))
		feFile.write('\n')
		feFile.close()
		
		hyperDistance = abs(dot(W, asarray(self.expertPolicyFE)-asarray(lastFE)))
		self.policyFEList.append(lastFE)
		if hyperDistance < self.minimumT:
			self.minimumT = hyperDistance
			self.bestPolicy = lastFE
		return hyperDistance
		
	def optimalWeightFinder(self):
		while True:
			W = self.optimization() # update only upon finding a closer point
			print ("weights ::", W )
			self.currentT = self.policyListUpdater(W)
			print ("Current distance (t) is:: ", self.currentT )
			if self.currentT <= self.epsilon:
				break
			self.iter = self.iter + 1
		return W
	
	def old_optimization(self):
		m = len(self.expertPolicyFE)
		P = matrix(2.0*eye(m), tc='d') # min ||w||
		q = matrix(zeros(m), tc='d')
		#G = matrix((matrix(self.expertPolicyFE) - matrix(self.randomPolicyFE)), tc='d')
		policyList = [self.expertPolicyFE]
		h_list = [1]
		for i in self.policiesFE:
			policyList.append(self.policiesFE[i])
			h_list.append(1)
		policyMat = array(policyList)
		policyMat[0] = -1*policyMat[0]
		print policyMat
		G = matrix(policyMat, tc='d')
		h = matrix(-array(h_list), tc='d')
		solvers.options['show_progress'] = False
		sol = solvers.qp(P,q,G,h)

		weights = squeeze(asarray(sol['x']))
		norm = linalg.norm(weights)
		weights = weights/norm
		return weights
				
	def optimization(self):
		policyMat = array(self.policyFEList)-self.expertPolicyFE
		print policyMat

		m = len(self.expertPolicyFE)
		n = len(self.policyFEList)
		P = matrix(concatenate((concatenate((2*eye(m),zeros((m,n))),axis = 1),zeros((n,n+m)))), tc='d') # min ||w|| - sum(eta_i)
		q = matrix(concatenate((zeros(m),ones(n))), tc='d')


		
		G = matrix(concatenate((policyMat,-eye(n)),axis=1), tc='d')
		h = matrix(array([-1]*len(policyMat)), tc='d')
		solvers.options['show_progress'] = False
		sol = solvers.qp(P,q,G,h)

		weights = squeeze(asarray(sol['x']))[:m]
		# norm = linalg.norm(weights)
		# weights = weights/norm
		return weights
			
			
if __name__ == '__main__':
	#rlEpisodes = 200
	#rlMaxSteps = 250
	#W = [-0.9, -0.9, -0.9, -0.9, 1]
	#env = gym.make('CartPole-v0')
	irlearner = irlAgent()
	#print irlearner.policiesFE
	#irlearner.policyListUpdater(W)
	#print irlearner.rlAgentFeatureExpecs(W)
	#print irlearner.expertFeatureExpecs()
	
	#print irlearner.optimization(20)
	#squeeze(asarray(M))
	# print (irlearner.QTrainer([0.264772755303, 0.414378964561, -0.331198312114, 0.480262962065, 0.637158522516, 0.0293366603083, 0.0729012788753, -0.0754613025673]))
	# print (irlearner.QTrainer([0.264772755303, 0.414378964561, -0.331198312114, 0.480262962065, 0.637158522516, 0.0729012788753, -0.0754613025673]))

	#print irlearner.optimalWeightFinder()
	#irlearner.QTrainer([0.35010781, 0.46186009, -0.21819772, 0.42189656, 0.56911922, 0.06891261, 0.31893225, -0.09021865])
        irlearner.QTrainer([0.35010781, 0 , 0 , -0.09021865])

