import sympy as sp
import DH 

"""
Erros Used in the code: (Note that the variation 6 is equal to 8, and 7 is equal to 9)
Simulation 1: Error value in mm: 0.0
'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0,}
'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0,}
'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0,}

Simulation 2: Error value in mm: 3.494246063248953 - Related.
'errors': {'sigma': 1.02469, 'beta': 0.0181, 'epsilon': 0.3606, 'phi': 0,}
'errors': {'sigma': 0.76852, 'beta': 0.0181, 'epsilon': 0.3606, 'phi': 0,}
'errors': {'sigma': 0.51235, 'beta': 0.0181, 'epsilon': 0.3606, 'phi': 0,}

"""

if __name__ == "__main__":
    
    #Define the DH parameters:(Three-link planar arm) 1,2,3,4,5,6,7
    Three_link_planar_arm = [
        {'type': 'revolute', 'a': 80, 'alpha': 0, 'd': 0,
         'errors': {'sigma': 1.02469, 'beta': 0.0181, 'epsilon': 0.3606, 'phi': 0,}},

        {'type': 'revolute', 'a': 60, 'alpha': 0, 'd': 0,
         'errors': {'sigma': 0.76852, 'beta': 0.0181, 'epsilon': 0.3606, 'phi': 0,}},
         
        {'type': 'revolute', 'a': 40, 'alpha': 0, 'd': 0, 
         'errors': {'sigma': 0.51235, 'beta': 0.0181, 'epsilon': 0.3606, 'phi': 0,}},
    ]
    
    #Create the robot:
    robot = DH.Mechanism(Three_link_planar_arm)
        
    #To plot the robot, we need to provide the values of the variables. Including other parameters that are not theta
    variable_values = {
    #Variables not provided firstly:
    
        
    #Variables of movement:
    robot.theta[0]: (sp.pi/12),   #theta_0 for the first cylindrical joint 15 degrees
    robot.theta[1]: (sp.pi/6),    #theta_1 for the second cylindrical joint 30 degrees
    robot.theta[2]: (sp.pi/4),    #theta_2 for the third cylindrical joint 45 degrees

    #Variables of errors:

    } 

    #robot.plot_mechanism( title ='Manipulador em Estado Inicial',initial_config=True, plot_type= '2d') 

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
    