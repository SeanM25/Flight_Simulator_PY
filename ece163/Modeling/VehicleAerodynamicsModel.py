import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel as VDM
from ..Modeling import WindModel
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleAerodynamicsModel:

    def __init__(self, initialSpeed = VPC.InitialSpeed, initialHeight = VPC.InitialDownPosition):

        '''Initialization of the internal classes which are used to track the vehicle aerodynamics and dynamics.'''

        # Check This For Typo?

        self.VDynamics = VDM.VehicleDynamicsModel() # Assign self all the parameters of the vehicle state

        self.VDynamics.state.u = initialSpeed # Velocity in x-dir equals the inital speed this assumes the plane is flying straight and level

        self.VDynamics.state.pd = initialHeight # the initial down position is the intial height

        return # Return nothing
    


    
    def CalculateCoeff_alpha(self, alpha):

        # CL & CD Blending Equation From Lecture

        blender_num = 1 + math.exp(-1 * VPC.M * (alpha - VPC.alpha0)) + math.exp(VPC.M * (alpha + VPC.alpha0)) # Numerator of the blending function

        blender_den = (1 + math.exp(-1 * VPC.M * (alpha - VPC.alpha0))) * (1 + math.exp(VPC.M * (alpha + VPC.alpha0)))  # Denominator of the blending function

        sigma = (blender_num / blender_den) # The sigmal value is the blending function numerator over denominator


        # CL (Coefficent of Lift) Equations for Attached (Pre-Stall) and for Seperated (Post-Stall) conditions 
        
        CL_attach = VPC.CL0 + (VPC.CLalpha * alpha) # CL attached equation from lecture, handouts, etc

        CL_sep = 2 * math.sin(alpha) * math.cos(alpha) # CL seperated equation from lab manual


        # CD (Coefficent of Drag) Equations for Attached (Pre-Stall) and for Seperated (Post-Stall) conditions 

        CD_attach_num = (VPC.CL0 + (VPC.CLalpha * alpha)) * (VPC.CL0 + (VPC.CLalpha * alpha)) # Numerator of CD_attached squared

        CD_attach = VPC.CDp + (CD_attach_num / (math.pi * VPC.AR * VPC.e)) # CD attached equation

        CD_sep = (2  * (math.sin(alpha) ** 2))  # CD seperated equation 2 sin^2 alpha


        # CL and CD total and CM equations

        CL_tot = ((1 - sigma) * (CL_attach)) + ((sigma) * (CL_sep)) # Given overall CL equation from handout

        CD_tot = ((1 - sigma) * (CD_attach)) + ((sigma) * (CD_sep)) # Given overall CD equation from handout

        CM_tot = VPC.CM0 + (VPC.CMalpha * alpha) # Given overall CM equation from handout


        return CL_tot, CD_tot, CM_tot # Return coefficents of Lift, Drag & Moment
    



    def CalculatePropForces(self, Va, Throttle):
    
        ''' Function to calculate the propeller forces and torques on the aircraft. 
        Uses the fancy propeller model that parameterizes the torque and thrust coefficients of the propeller using the advance ratio. 
        See ECE163_PropellerCheatSheet.pdf for details. Note: if the propo speed Omega is imaginary, then set it to 100.0 '''
    

    # We need omega for poth the propellor force and the torque so we should first find that

        # Need KT and KV to find omega via the quadratic formula specified in the prop cheat sheet

        KT = (60 / (2 *  math.pi * VPC.KV)) # KT equals the equation directly below (6) in the Prop cheat sheet

        KE = KT # KE = KT according the equation

        # Need Vin as well Vin = Vmax * throttle value according to blurb in the prop cheat sheet

        Vin = VPC.V_max * Throttle

        # Assemble the quadratic equation values a, b, c

        a = ((VPC.rho) * (VPC.D_prop ** 5) * (VPC.C_Q0)) / (4 * math.pi ** 2) # Given a

        b = (((VPC.rho) * (VPC.D_prop ** 4) * (Va) * (VPC.C_Q1)) / (2 * math.pi)) + ((KT * KE) / (VPC.R_motor)) # Given b

        c = ((VPC.rho) * (VPC.D_prop ** 3) * (Va ** 2) * (VPC.C_Q2)) - (KT * (Vin/VPC.R_motor)) + (KT * VPC.i0) # Given c

        # Check for Imaginary omega

        if ((b ** 2) < (4 * a * c)): # For the quadratic formula if b^2 < 4ac then omega must be an imaginary num that is omega = (- b +/- j / 2a )

            omega = 100.0 # If imaginary make omega 100

        else:

            omega = ((-1 * b) + math.sqrt((b ** 2) - (4 * a * c)) / (2 * a)) # Otherwise calculate omega using the quadratic formula normally

        # Get J to find CT & CQ

        J = ((2 * math.pi * Va) / (omega * VPC.D_prop))

        # Get CQ & CT for final calculation

        CT = VPC.C_T0 + (VPC.C_T1 * J) + (VPC.C_T2 * (J ** 2)) # Equation (3)

        CQ = VPC.C_Q0 + (VPC.C_Q1 * J) + (VPC.C_Q2 * (J ** 2)) # Equation (4)

        
        Fx = (((VPC.rho) * (omega ** 2) * (VPC.D_prop ** 4) * (CT)) / (4 * (math.pi ** 2))) # Equation (1) Force of Prop

        Mx = (((VPC.rho) * (omega ** 2) * (VPC.D_prop ** 5) * (CQ)) / (4 * (math.pi ** 2))) # Equation (2) Moment of prop

        return Fx, Mx # Return Prop Force and Moment



    def setVehicleState(self, state):

        '''Wrapper function to set the vehicle state from outside module'''

        self.VDynamics.state = state # Set Vehicle State to current state

        return # Return nothing



    def getVehicleState(self):

        '''Wrapper function to return vehicle state from module'''

        return self.VDynamics.state # Return current Vehicle state


    def getVehicleDynamicsModel(self):


        '''Wrapper function to return the vehicle dynamics model handle'''

        return self.VDynamics # Return vehicle dynamics model which is set to the Vehicle Dynamics Module class

    def reset(self):

        '''Resets module to its original state so it can run again'''

        # Basically Just Copy what was in init since were resetting

        self.VDynamics = VDM.VehicleDynamicsModel() # Reset to vanilla vehicle dynamics model

        self.VDynamics.state.u = VPC.InitialSpeed # Reset intial Speed to default

        self.VDynamics.state.pd = VPC.InitialDownPosition # Reset height to default

        return # return nothing
        
    def gravityForces(self, state):

        grav_vect = [[0], [0], [VPC.mass * VPC.g0]] # 0, 0, mg gravity vector

        F_grav = mm.multiply(state.R, grav_vect) # Multiply rotation matrix by grav vector to get F grav according to lecture

        Fx = F_grav[0][0] # Gravity in the x should be 0

        Fy = F_grav[1][0] # Gravity in the y should be 0

        Fz = F_grav[2][0] # Gravity in the z 

        tot_gravity = Inputs.forcesMoments(Fx, Fy, Fz) # Gravity is a forces moments class

        return tot_gravity # return gravity



    def controlForces(self, state, controls):


        '''Function to calculate aerodynamic forces from control surface deflections (including throttle) using the linearized aerodynamics and simplified thrust model.
           Requires airspeed (Va) in [m/s] and angle of attack (alpha) in [rad] both from state.Va and state.alpha respectively.'''

        force_const = (1 / 2) * (VPC.rho) * (state.Va ** 2) * VPC.S # constant term that exists in Force of Lift, Drag, etc equations

        R_Fx_Fz_cont = [[math.cos(state.alpha), -1 * math.sin(state.alpha)],  # Given matrix needed to get Fx, Fz this time with control defkections
                        [math.sin(state.alpha), math.cos(state.alpha)]]

        F_drag_cont = force_const * (VPC.CDdeltaE * controls.Elevator) # Beards drag equation but only for control surfaces

        F_lift_cont = force_const * (VPC.CLdeltaE * controls.Elevator) # Beards lift equation but only for control surfaces

        Drag_Lift_Vec = [[-1 * F_drag_cont], [-1 * F_lift_cont]] # Same vector as before but this time exculsively for control surfaces

        DL_cont = mm.multiply(R_Fx_Fz_cont, Drag_Lift_Vec) # Multiply R by lift drag to get Fx, Fz for control surfaces

        Fx_cont = DL_cont[0][0] # Assign Fx for controls

        Fy_cont = force_const * ((VPC.CYdeltaA * controls.Aileron) + (VPC.CYdeltaR * controls.Rudder)) # Beard's Fy for control surfaces

        Fz_cont = DL_cont[1][0] # Assign Fz for controls

        Mx_cont = force_const * VPC.b * ((VPC.CldeltaA * controls.Aileron) + (VPC.CldeltaR * controls.Rudder)) # Roll Moment due to control input

        My_cont = force_const * VPC.c * (VPC.CMdeltaE * controls.Elevator) # Pitch Moment due to control input

        Mz_cont = force_const * VPC.b * ((VPC.CndeltaA * controls.Aileron) + (VPC.CndeltaR * controls.Rudder)) # Yaw Moment due to control input

        # Need propellor forces since were also asked to use throttle this effects the Forces in x only


        force_from_prop, moments_from_prop = VehicleAerodynamicsModel.CalculatePropForces(self, state.Va, controls.Throttle)

        Fx = Fx_cont + force_from_prop # Total amount of forces in X for controls is due to both prop and control surfaces

        Fy = Fy_cont # Amount in Y for controls is just due to control surfaces

        Fz = Fz_cont # Fz equal to forces of control surfaces in Z

        Mx = Mx_cont + moments_from_prop # Moments from Propellor and the control surface deflections

        My = My_cont # Moments in Y for control surface deflections

        Mz = Mz_cont # Moments in Z for control surface deflections

        control_forces = Inputs.forcesMoments(Fx, Fy, Fz, Mx, My, Mz) # Assign control surfaces

        return control_forces # Return control surfaces




    def updateForces(self, state, controls, wind = None):

        '''Function to update all of the aerodynamic, propulsive, and gravity forces and moments. All calculations required to update the forces are included. 
           state is updated with new values for airspeed, angle of attack, and sideslip angles (see class definition for members)'''



        gravF = VehicleAerodynamicsModel.gravityForces(self, state) # Get gravity forces

        aeroF = VehicleAerodynamicsModel.aeroForces(self, state) # Get aero forces

        controlF = VehicleAerodynamicsModel.controlForces(self, state, controls) # Get forces due to control deflection

        Fx = gravF.Fx + aeroF.Fx + controlF.Fx # Sum of all forces in X due to gravity, aeroforces, and control forces

        Fy = gravF.Fy + aeroF.Fy + controlF.Fy # Sum of all forces in Y due to gravity, aeroforces, and control forces

        Fz = gravF.Fz + aeroF.Fz + controlF.Fz # Sum of all forces in Z due to gravity, aeroforces, and control forces


        Mx = gravF.Mx + aeroF.Mx + controlF.Mx # Sum of all moments in X due to gravity, aeroforces, and control forces

        My = gravF.My + aeroF.My + controlF.My # Sum of all forces in Y due to gravity, aeroforces, and control forces

        Mz = gravF.Mz + aeroF.Mz + controlF.Mz # Sum of all forces in Z due to gravity, aeroforces, and control forces

        updatedForce = Inputs.forcesMoments(Fx, Fy, Fz, Mx, My, Mz) # All forces updated

        return updatedForce # Return updated forces
    
    def Update(self, controls):


        '''Function that uses the current state (internal), wind (internal), and controls (inputs) to calculate the forces, and then do the integration of the full 6-DOF non-linear equations of motion.
          Wraps the VehicleDynamicsModel class as well as the windState internally. The Wind and the vehicleState are maintained internally.'''
        
        state = VehicleAerodynamicsModel.getVehicleState(self) # Get present vehicle state

        updated_forces = VehicleAerodynamicsModel.updateForces(self, state, controls, wind = None) # Get current forces on plane

        self.VDynamics.Update(updated_forces) # upadate the forces on our model

        return # return nothing
    

    def aeroForces(state):

        '''Function to calculate the Aerodynamic Forces and Moments using the linearized simplified force model
        and the stability derivatives in VehiclePhysicalConstants.py file. 
        Specifically does not include forces due to control surface deflection.
        Requires airspeed (Va) in [m/s], angle of attack (alpha) in [rad] and sideslip angle (beta) in [rad] from the state.'''

        # Need Fx, Fy, Fz, Mx, My, Mz to get all aero forces and moments

        if(state.Va == 0): # If there is no airspeed the aircraft is not flying and there are no forces acting upon it

            Fx = 0.0 # No X force aaa

            Fy = 0 # No Y force

            Fz = 0 # No Z force

            Mx = 0 # No X moment

            My = 0 # No Y moment

            Mz = 0 # No Z moment

        else: # Do all the calculations and get the forces and moments
        
        # F_Drag & F_Lift equations no control surface deflection. Get Fx, Fz

        
            CL_alpha, CD_alpha, CM_alpha = VehicleAerodynamicsModel.CalculateCoeff_alpha(self, state.alpha) # Get Coefficents of Lift and Drag to do the calculations
            
            force_const = (1 / 2) * VPC.rho * (state.Va ** 2) * VPC.S # constant term that exists in Force of Lift, Drag, etc equations

            q_term = (VPC.c * state.q) / (2 * state.Va) # Constant term within Flift and drag we multiply by q

            p_term = (VPC.b * state.p) / (2 * state.Va) # Constant term within moments we multiply by p

            r_term = (VPC.b * state.r ) / (2 * state.Va) # Constant term within moments we multiply by r

            R_Fx_Fz = [[math.cos(state.alpha), -1 * math.sin(state.alpha)],  # Given matrix needed to get Fx, Fz
                       [math.sin(state.alpha), math.cos(state.alpha)]]
        
            F_drag = (force_const * (CD_alpha + (VPC.CDq * q_term))) # Given Drag eq

            F_lift = (force_const * (CL_alpha + (VPC.CLq * q_term))) # Given lift eq

            vec_lift_drag = [[-1 * F_drag], [-1 * F_lift]]  # R times this vector gives us Fx, Fz

            vec_Fx_Fz = mm.multiply(R_Fx_Fz, vec_lift_drag) # Get Fx, Fz vector

            # Get Fy using Eq 4.14 from our pal Beard

            Fy = (force_const * (VPC.CY0 + (VPC.CYbeta * state.beta) + (VPC.CYp * p_term) + (VPC.CYr * r_term))) # Assign Fy

            Fx = vec_Fx_Fz[0][0] # assign Fx

            Fz = vec_Fx_Fz[1][0] # assign Fz

            # Get The Moments Mx, My, Mz

            Mx = ((force_const * VPC.b) * (VPC.Cl0 + (VPC.Clbeta * state.beta) + (VPC.Clp * p_term) + (VPC.Clr * r_term))) # Eq 4.15 Beard Roll Moment

            My = ((force_const * VPC.c) * (VPC.CM0 + (VPC.CMalpha * state.alpha) + (VPC.CMq * q_term))) # Eq 4.5 Beard Pitch Moment

            Mz = ((force_const * VPC.b) * (VPC.Cn0 + (VPC.Cnbeta * state.beta) + (VPC.Cnp * p_term) + (VPC.Cnr * r_term))) # Eq 4.16 Beard Yaw Moment
        
            # Gather all aeroforces

            AForce = Inputs.forcesMoments(Fx, Fy, Fz, Mx, My, Mz)

            # Return Aeroforces

            return AForce

