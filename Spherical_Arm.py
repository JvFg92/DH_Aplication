import sympy as sp
import DH 

if __name__ == "__main__":
    
    #Define the DH parameters:(exempli gratia: Spherical Arm)
    Spherical_Arm = [
        {'type': 'revolute', 'a': 0, 'alpha': -sp.pi/2, 'd': 0.0,
         'errors': {'phi': 0.05, 'epsilon': 0.02, 'sigma': 0.01, 'beta': 0.03}},
        
        {'type': 'revolute', 'a':0, 'alpha': sp.pi/2, 
         'errors': {'phi': 0.03, 'epsilon': 0.01, 'sigma': 0.02, 'beta': 0.02}},

        {'type': 'prismatic', 'a': 0, 'alpha': 0, 'theta': 0,
         'errors': {'phi': 0.0, 'epsilon': 0.05, 'sigma': 0.0, 'beta': 0.0}}
    ]

    #Create the robot:
    robot = DH.Mechanism(Spherical_Arm)
    
    """Begin the calculations algebrically"""

    #Calculate the forward kinematics without errors:
    matrix_g,position_g=robot.forward_kinematics(False)
    print("\nMatrix and position algebrical without errors:\n")
    print("\nMatrix:\n", matrix_g)
    print("\nPosition:\n", position_g)
    
    
    #Calculate the forward kinematics with errors:
    matrix_g_e,position_g_e=robot.forward_kinematics(True)
    print("\nMatrix and position algebrical with errors:\n")
    print("Matrix:\n", matrix_g_e)
    print("\nPosition:\n", position_g_e)

    """Begin the calculations numerically:"""

    #Calculate the forward kinematics without errors & DH parameters applied:
    matrix_g_numerical, position_g_numerical = robot.evaluate_param(matrix_g)
    print("\nMatrix and position numerical without errors:\n")
    print("\nMatrix:\n", matrix_g_numerical)
    print("\nPosition:\n", position_g_numerical)

 
    #Validating the matrix:
    for i in range(matrix_g_numerical.shape[0]):
        for j in range(matrix_g_numerical.shape[1]):
            print(f"\n Element ({i},{j}):", matrix_g_numerical[i,j], end=" ")
    print("\n")

    
    #Calculate the forward kinematics with errors & DH parameters applied:
    matrix_g_numerical_e, position_g_numerical_e = robot.evaluate_param(matrix_g_e, apply_errors=True)
    print("\nMatrix and position numerical with errors:\n")
    print("Matrix:\n", matrix_g_numerical_e)
    print("\nPosition:\n", position_g_numerical_e)
    

    """Start calculations numerically with optional variable values ​​(graphical example):"""
    
    #Evaluate the position of the effector using optional variable values:
    """To plot the robot, we need to provide the values of the variables. including other parameters that are not 'theta' or 'd'"""
    variable_values = {
    #Variables not provided (Defined in the DH parameters, for plot eg):
    robot.d[1]:     1,
    #Variables of movement:
    robot.theta[0]: 0.5,    #theta_0 for the first cylindrical joint
    robot.theta[1]: 1.0,    #theta_1 for the second cylindrical joint   
    robot.d[2]:     2.0     #d_2 for the prismatic joint
    }    
    
    #Calculate the forward kinematics without errors DH parameters applied:
    matrix_numerical, position_numerical = robot.evaluate_param(matrix_g)
    print("\nMatrix and position numerical without errors:\n")
    print("\nMatrix:\n", matrix_numerical)
    print("\nPosition:\n", position_numerical)

    #Calculate the forward kinematics with errors DH parameters applied:
    matrix_numerical_e, position_numerical_e = robot.evaluate_param(matrix_g_e, apply_errors=True)
    print("\nMatrix and position numerical with errors:\n")
    print("Matrix:\n", matrix_numerical_e)
    print("\nPosition:\n", position_numerical_e)

    #Calculate error values:
    print("\n [X, Y, Z] position values:")
    print("Position without erros: \n", [p for p in position_numerical])
    print("Position with errors: \n", [p for p in position_numerical_e])
    error = [0, 0, 0]
    for i in range(3):
        error[i] = position_numerical_e[i] - position_numerical[i]
    print("\n [X, Y, Z] error values:")
    print("\n \n", error)
        
    #Plot the robot:
    robot.plot_mechanism(variable_values, title ='Mechanism kinematics with and without errors')
    