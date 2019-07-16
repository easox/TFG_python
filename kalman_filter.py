import numpy as np



class KalmanFilter():

    def init_pos_vel_accel_x(self):
      #x=[x,y,v_x,v_y,a_x,a_y]

          
      self.A=np.array([[1 ,0, self.T, 0 ,self.T*self.T/2, 0],
                  [0 ,1, 0, self.T, 0, self.T*self.T/2],
                  [0, 0 ,1, 0, self.T, 0],
                  [0, 0, 0 ,1, 0, self.T],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]]
                  )

      self._x=np.array([[0],[0],[0],[0],[0],[0]])

       #define P with some uncertainty in x,y but low in v and accel. Tell user to press button when still
       #TODO: ADD uncertainty to accel since it can change at t+1
      self._P=np.array([[5**2,0,0,0,0, 0],
                       [0,5**2,0,0,0, 0],
                       [ 0,  0 ,self.sigma_vel_x**2,self.sigma_vel_xy**2,0,0],
                       [ 0,  0, self.sigma_vel_xy**2,self.sigma_vel_y**2,0,0],
                       [ 0,  0, 0, 0,self.sigma_accel_x**2,self.sigma_accel_xy**2],
                       [ 0,  0, 0, 0, self.sigma_accel_xy**2,self.sigma_accel_y**2]])


      #TODO:tune values of Q

      self._Q=np.array([[self.sigma_accel_x**2*self.T**4/4,0,self.sigma_accel_x**2*self.T**3/2,0,self.sigma_accel_x**2*self.T**2/2, 0],
                       [0, self.sigma_accel_y**2*self.T**4/4,0,self.sigma_accel_y**2*self.T**3/2,0,self.sigma_accel_y**2*self.T**2/2],
                       [self.sigma_accel_x**2*self.T**3/2,0,self.sigma_accel_x**2*self.T**2,0,self.sigma_accel_x**2*self.T, 0],
                       [0,self.sigma_accel_y**2*self.T**3/2,0,self.sigma_accel_y**2*self.T**2,0,self.sigma_accel_y**2*self.T],
                       [self.sigma_accel_x**2*self.T**2/2,0,self.sigma_accel_x**2*self.T,0,self.sigma_accel_x**2*1, 0],
                       [0,self.sigma_accel_y**2*self.T**2/2,0,self.sigma_accel_y**2*self.T,0,1]])
      
    def init_accel_y(self):
      #y=[a_x,a_y]
      self._H=np.diag([0,0,0,0,1,1])
      
     # self.R=self.R[[4:6,4:]]

    def init_pos_accel_y(self):
      #y=[x,y,a_x,a_y]
      self._H=np.diag([1,1,0,0,1,1])
      
      #self.R=self.R[[0,1,4,5],[0,1,4,5]]
    

    def init_pos_vel_accel_y(self):
      #y=[x,y,v_x,v_y,a_x,a_y]
               
      self._H=np.eye(6)

      #TODO: 
      #Get R data to parse (initialized with class)
      self.R=self.R


    def __init__(self, sigma_accel_x,sigma_accel_y,sigma_pos_x,sigma_pos_y,sigma_vel_x,sigma_vel_y,sigma_pos_xy,sigma_vel_xy,sigma_accel_xy,T):
        # Model parameters
        self.sigma_accel_x=sigma_accel_x
        self.sigma_vel_x=sigma_vel_x
        self.sigma_pos_x=sigma_pos_x

        self.sigma_accel_y=sigma_accel_y
        self.sigma_vel_y=sigma_vel_y
        self.sigma_pos_y=sigma_pos_y

        self.sigma_accel_xy=sigma_accel_xy
        self.sigma_vel_xy=sigma_vel_xy
        self.sigma_pos_xy=sigma_pos_xy

        self.R=np.array([[sigma_pos_x**2,sigma_pos_xy**2,0,0,0,0],
                         [sigma_pos_xy**2,sigma_pos_y**2,0,0,0,0],
                         [0,0,sigma_vel_x**2,sigma_vel_xy**2,0,0],
                         [0,0,sigma_vel_xy**2,sigma_vel_y**2,0,0],
                         [0,0,0,0,sigma_accel_x**2,sigma_accel_xy**2],
                         [0,0,0,0,sigma_accel_xy**2,sigma_accel_y**2]])
        
        self.T=T

    def predict(self):
        self._x = self.A @ self._x
        self._P = self.A @ self._P @ self.A.transpose() + self._Q

    def update(self, z):
        self.S = self._H @ self._P @ self._H.transpose() + self.R
        self.V = z - self._H @ self._x
        self.K = self._P @ self._H.transpose() @ np.linalg.inv(self.S)

        self._x = self._x + self.K @ self.V
        self._P = self._P - self.K @ self.S @ self.K.transpose()

    def get_state(self):
        return self._x, self._P