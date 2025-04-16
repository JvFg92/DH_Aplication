import sympy as sp
import DH 

if __name__ == "__main__":
    
    #Define the DH parameters:(exempli gratia: Spherical Arm)
    Spherical_Arm = [
        {'type': 'revolute', 'a': 0, 'alpha': -sp.pi/2, 'd': 0.0,
         'errors': {'phi': 0.05, 'epsilon': 0.02, 'sigma': 0.01, 'beta': 0.03}},
        
        {'type': 'revolute', 'a':0, 'alpha': sp.pi/2,  'd': 1.0,
         'errors': {'phi': 0.03, 'epsilon': 0.01, 'sigma': 0.02, 'beta': 0.02}},

        {'type': 'prismatic', 'a': 0, 'alpha': 0, 'theta': 0,
         'errors': {'phi': 0.0, 'epsilon': 0.05, 'sigma': 0.0, 'beta': 0.0}}
    ]

    #Create the robot:
    robot = DH.Mechanism(Spherical_Arm)
    
    #Evaluate the position of the effector using optional variable values:
    """To plot the robot, we need to provide the values of the variables. including other parameters that are not 'theta' or 'd'"""
    variable_values = {
    #Variables not provided (Defined in the DH parameters, for plot eg):
    
    #Variables of movement:
    robot.theta[0]: 0.5,    #theta_0 for the first cylindrical joint
    robot.theta[1]: 1.0,    #theta_1 for the second cylindrical joint   
    robot.d[2]:     2.0     #d_2 for the prismatic joint
    } 
    
    #robot.plot_mechanism( title ='Manipulador em Estado Inicial',initial_config=True) 

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

    #Calculate the forward kinematics without errors DH parameters applied:
    matrix_g_numerical, position_g_numerical, orientation_g_numerical = robot.evaluate_param(matrix_g)
    print("\nMatrix and position numerical without errors:(Book Validation)\n")
    print("\nMatrix:\n", matrix_g_numerical)
    print("\nPosition:\n", position_g_numerical)
    print("\nOrientation:\n", orientation_g_numerical)

    #Calculate the forward kinematics with errors DH parameters applied:
    matrix_ge_numerical, position_ge_numerical, orientation_ge_numerical = robot.evaluate_param(matrix_g_e,apply_errors=True)
    print("\nMatrix and position numerical with errors:(Book Validation)\n")
    print("\nMatrix:\n", matrix_ge_numerical)
    print("\nPosition:\n", position_ge_numerical)
    print("\nOrientation:\n", orientation_ge_numerical)

    '''Results for research purposes'''
    
    #Calculate the forward kinematics with errors DH parameters applied:
    matrix_numerical, position_numerical, orientation_numerical = robot.evaluate_param(matrix_g,variable_values)
    print("\nMatrix and position numerical without errors:\n")
    print("\nMatrix:\n", matrix_numerical)
    print("\nPosition:\n", position_numerical)
    print("\nOrientation:\n", orientation_numerical)

    #Calculate the forward kinematics with errors DH parameters applied:
    matrix_numerical_e, position_numerical_e, orientation_numerical_e = robot.evaluate_param(matrix_g_e, variable_values, apply_errors=True)
    print("\nMatrix and position numerical with errors:\n")
    print("Matrix:\n", matrix_numerical_e)
    print("\nPosition:\n", position_numerical_e)
    print("\nOrientation:\n", orientation_numerical_e)

    #Calculate error values:
    print("\n [X, Y, Z] position values:")
    print("Position without erros: \n", [p for p in position_numerical])
    print("Position with errors: \n", [p for p in position_numerical_e])
    error=robot.evaluate_error()
    print("\n Error value in mm: \n", error)

    #Calculate the euler angles:
    print("\nEuler angles:Yaw(X),Pitch(Y),Roll(Z)")
    print("Without errors: \n", robot.get_euler_angles())
    print("With errors: \n", robot.get_euler_angles(apply_errors=True))
        
    #Plot the robot:
    #CERTIFY ALL THE VARIABLES ARE PROVIDED BEFORE PLOTTING#
    robot.plot_mechanism(variable_values, title ='Robô 3R', plot_type= '3d')