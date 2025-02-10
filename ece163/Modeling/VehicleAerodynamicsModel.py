import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel as VDM
from ..Modeling import WindModel as WM
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleAerodynamicsModel:

    def __init__(self, initialSpeed = VPC.InitialSpeed, initialHeight = VPC.InitialDownPosition):

        '''Initialization of the internal classes which are used to track the vehicle aerodynamics and dynamics.'''

        self.VDynamics = VDM.VehicleDynamicsModel() # Assign self all the parameters of the vehicle state

        self.VDynamics.state.u = initialSpeed # Velocity in x-dir equals the inital speed this assumes the plane is flying straight and level

        self.VDynamics.state.pd = initialHeight # the initial down position is the intial height

       
    # New Intializations 4 Wind Model (Don't Want Wind? comment out):

        self.WindModel = WM.WindModel() # Initialize Wind Model Params

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

        KT = KE = 60 / (2 *  math.pi * VPC.KV) # KT equals the equation directly below (6) in the Prop cheat sheet

        # Need Vin as well Vin = Vmax * throttle value according to blurb in the prop cheat sheet

        Vin = VPC.V_max * Throttle

        # Assemble the quadratic equation values a, b, c

        a = (VPC.rho * (VPC.D_prop ** 5) * VPC.C_Q0) / (4 * (math.pi ** 2)) # Given a

        b = ((VPC.rho * (VPC.D_prop ** 4) * Va * VPC.C_Q1) / (2 * math.pi)) + ((KT * KE) / (VPC.R_motor)) # Given b

        c = (VPC.rho * (VPC.D_prop ** 3) * (Va ** 2) * VPC.C_Q2) - (KT * ((Vin) / VPC.R_motor)) + (KT * VPC.i0) # Given c

        # Check for Imaginary omega

        if ((b ** 2) < (4 * a * c) or (b ** 2) == (4 * a * c)): # For the quadratic formula if b^2 < 4ac then omega must be an imaginary num that is omega = (- b +/- j / 2a )

            omega = 100 # If imaginary make omega 100

        else:

            omega = ((-1 * b) + math.sqrt((b ** 2) - (4 * a * c))) / (2 * a) # Otherwise calculate omega using the quadratic formula normally

        # Get J to find CT & CQ

        J = (2 * math.pi * Va) / (omega * VPC.D_prop)

        # Get CQ & CT for final calculation

        CT = VPC.C_T0 + (VPC.C_T1 * J) + (VPC.C_T2 * (J ** 2)) # Equation (3)

        CQ = VPC.C_Q0 + (VPC.C_Q1 * J) + (VPC.C_Q2 * (J ** 2)) # Equation (4)

        
        Fx_propel = (VPC.rho * (omega ** 2) * (VPC.D_prop ** 4) * CT) / (4 * (math.pi ** 2)) # Equation (1) Force of Prop

        Mx_propel = (-1) * ((VPC.rho * (omega ** 2) * (VPC.D_prop ** 5) * CQ) / (4 * (math.pi ** 2))) # Equation (2) Moment of prop

        return Fx_propel, Mx_propel # Return Prop Force and Moment



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


        # Wind Model Reset

        self.WindModel = WM.WindModel() # Reset Wind Model conditions

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

        # Update Va, alpha, beta with wind this time

        Va, alpha, beta = self.CalculateAirspeed(state, wind) # Get Va, Alpha, and beta from the airspeed function which factors in Wind

        '''

        Prev No Wind Code:

        Va = math.hypot(state.u, state.v, state.w) # Va equation with no wind from lecture

        alpha = math.atan2(state.w, state.u) # Given equation for angle of attack no wind cond

        if (math.isclose(Va, 0.0)): # If Va is near zero

            beta = 0 # Make it zero
        else:

            beta = math.asin((state.v) / (Va)) # Otherwise calculate it using no wind ocndition

        '''

        state.Va = Va # Assign Va

        state.alpha = alpha # Assign alpha

        state.beta = beta # Assign Beta
        
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

        wind = VehicleAerodynamicsModel.getWindModel(self).wind # Get Wind

        updated_forces = VehicleAerodynamicsModel.updateForces(self, state, controls, wind) # Get current forces on plane

        self.VDynamics.Update(updated_forces) # upadate the forces on our model

        

        return # return nothing
    

    def aeroForces(self, state):

        '''Function to calculate the Aerodynamic Forces and Moments using the linearized simplified force model
        and the stability derivatives in VehiclePhysicalConstants.py file. 
        Specifically does not include forces due to control surface deflection.
        Requires airspeed (Va) in [m/s], angle of attack (alpha) in [rad] and sideslip angle (beta) in [rad] from the state.'''

        

        # Need Fx, Fy, Fz, Mx, My, Mz to get all aero forces and moments

        

        if(state.Va == 0): # If there is no airspeed the aircraft is not flying and there are no forces acting upon it

            Fx = 0 # No X force aaa

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
    

    def getWindModel(self):

        '''Wrapper function to return the windModel'''

        return self.WindModel # Return current wind model
    
    def setWindModel(self, windModel):

        '''Wrapper function to set the windModel'''


        self.WindModel = windModel # set wind model to given wind model

        return # return nothing
    
    def CalculateAirspeed(self,state, wind):


        '''
        Calculates the total airspeed, as well as angle of attack and side-slip angles from the wind and current state. 
        Needed for further aerodynamic force calculations.
        Va, wind speed [m/s], alpha, angle of attack [rad], and beta, side-slip angle [rad] are returned from the function. 
        The state must be updated outside this function.

        Note: when total wind speed (math.hypot(wind.Wn, wind.We, wind.Wd)) is zero,
        set gamma wind to 0 (otherwise it is undefined because arcsin is greater than 1.),
        Also, check is the total airspeed (Va) is zero before calculating the sideslip angle beta, if so, set beta to 0.0
        
        '''

        # This function will follow what was outlined in the Wind Model slides from lecture

        # Va = Vg - Vw where wind speed must be determined

        # Get current inertial wind Speed

        W_steady = math.hypot(wind.Wn, wind.We, wind.Wd) # Calculate steady state wind speed

        # Calculate course angle

        X_w = math.atan2(wind.We, wind.Wn) # Course angle calculation

        # Calculate Gamma

        if(W_steady == 0):

            Gamma_w = 0 # If total wind speed is 0 gamma is also zero

        else:

            Gamma_w = -1 * math.asin(wind.Wd / W_steady) # Otherwise use the equation from lecture

        # Assemble R Azimuth Elevation

        R_AZEV = [[math.cos(X_w) * math.cos(Gamma_w), math.sin(X_w) * math.cos(Gamma_w), -1 * math.sin(Gamma_w)], 
                  [-1 * math.sin(X_w), math.cos(X_w), 0],
                  [math.cos(X_w) * math.sin(Gamma_w), math.sin(X_w) * math.sin(Gamma_w), math.cos(Gamma_w)]] # Result of multiplying the Azimuth and Elev matrices from lect
        

        # Get necessary Wind vectors for inertial, velocity frames, and ground speed

        Vg = [[state.u], [state.v], [state.w]] # Get current ground speed

        W_iner = [[wind.Wn], [wind.We], [wind.Wd]] # Get wind in inertial frame

        W_vel = [[wind.Wu], [wind.Wv], [wind.Ww]] # Get wind gusts in velocity frame

        # Get Wind Speed

        W_gusts_inertial = mm.multiply(mm.transpose(R_AZEV), W_vel) # Get wind gusts in inertial frame

        WS_tot_iner = mm.add(W_iner, W_gusts_inertial) # Get total Wind speed in inertial frame

        Ws_tot_bod = mm.multiply(state.R, WS_tot_iner) # Get total wind speed in the body frame [u_w, v_w, w_w]

        # Get airspeed vector

        Va_vector = mm.subtract(Vg, Ws_tot_bod) # Get airspeed vector we take the magnitude to get Va

        Ur = Va_vector[0][0] # Get U component

        Vr = Va_vector[1][0] # Get V component

        Wr = Va_vector[2][0] # Get W component

        # Calculate Va

        Va = math.hypot(Ur, Vr, Wr) # Calculate the airspeed

        # Get angle of attack

        alpha = math.atan2(Wr, Ur) # Beard Pg 57

        # Get Beta and check for 0

        if (math.isclose(Va, 0)): # If airspeed is close to zero

            beta = 0 # beta is 0
        else:

            beta = math.asin(Vr / Va) # Otherwise use equation from Beard pg 57


        return Va, alpha, beta # Return airspeed, angle of attack, and sideslip angle


        









