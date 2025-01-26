def derivative(self, state, forcesMoments):

        '''Function to compute the time-derivative of the state given body frame forces and moments'''

        # Derivatives NED (Pn, Pe, Pd)

        velocity_vector = [[state.u], [state.v], [state.w]] # Get u, v, w matrix

        R_transp = mm.transpose(state.R) # Transpose R inertial to body

        dot_pos_vector = mm.multiply(R_transp, velocity_vector) # multiply R trans by u,v,w to get the position derivative

        pn_dot = dot_pos_vector [0][0] # Get Pn dot

        pe_dot = dot_pos_vector [1][0] # Get Pe dot

        pd_dot = dot_pos_vector [2][0] # Get Pd dot

        # Derivatives of velocities (u, v, w)

        minus_wx_times_uvw = [[(state.r * state.v) - (state.q * state.w)], [(state.p * state.w) - (state.r * state.u)], [(state.q * state.u) - (state.p * state.v)]] # Given expanded matrix from lecture

        F_over_m = [[(forcesMoments.Fx) / (VPC.mass)] , [(forcesMoments.Fy) / (VPC.mass)], [(forcesMoments.Fz) / (VPC.mass)]] # Forces vector scaled by 1/m

        dot_UVW = mm.add(F_over_m, minus_wx_times_uvw) # derivative of the velocities vector

        u_dot = dot_UVW[0][0] # u dot
    
        v_dot = dot_UVW[1][0] # v dot

        w_dot = dot_UVW[2][0] # w dot




        # derivatives of yaw, pitch, roll

        yaw_pitch_roll = [[state.p], [state.q], [state.r]] # Given values of roll, pitch, and yaw

        YPR_dir_mtrx = [[1, (math.sin(state.roll) * math.tan(state.pitch)), (math.cos(state.roll) * math.tan(state.pitch))],
                        [0, math.cos(state.roll), -1 * math.sin(state.roll)],
                        [0, (math.sin(state.roll) / math.cos(state.pitch)), (math.cos(state.roll) / math.cos(state.pitch))]] # Given matrix from lecture
        
        dot_YPR = mm.multiply(YPR_dir_mtrx, yaw_pitch_roll) # get yaw pitch and roll derivatives

        roll_dot = dot_YPR[0][0] # Get roll dot

        pitch_dot = dot_YPR[1][0] # Get pitch dot
        
        yaw_dot = dot_YPR[2][0] # get yaw dot

        # Derivitive of UVW

        m_xyz = [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]] # Get current moments vector
        
        omega_cross = mm.skew(state.p, state.q, state.r) # skew symmetric

        pqr = [[state.p], [state.q], [state.r]] # get current pqr

        neg_J_inv = mm.scalarMultiply(-1, VPC.JinvBody) # Get -J^-1

        left_term = mm.multiply(VPC.JinvBody, m_xyz) # Multiply J times moments vector

        right_term = mm.multiply(mm.multiply(neg_J_inv, omega_cross), mm.multiply(VPC.Jbody, pqr)) # Multiply the other 4 terms together

        dot_pqr = mm.add(left_term, right_term) # Add them together

        p_dot = dot_pqr[0][0] # Get p dot

        q_dot = dot_pqr[1][0] # get q dot

        r_dot = dot_pqr[2][0] # get r dot

        # Derivative of R

        R_dot = mm.scalarMultiply(-1, (mm.multiply(omega_cross, state.R)))

        dot = States.vehicleState(pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot, yaw_dot, roll_dot, pitch_dot, p_dot, q_dot, r_dot, R_dot)

        return dot