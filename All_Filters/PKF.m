classdef PKF < kalman_filter_interface
    methods
        function b_next = estimate(b,U,Zg,lnr_sys_for_prd,lnr_sys_for_update)
            if nargin < 5
                error('Ali: The linearized systems has to be provided for LKF.')
            end
            b_prd = PKF.predict(b,U,lnr_sys_for_prd);
            b_next = PKF.update(b_prd,Zg,lnr_sys_for_update);
        end
        function b_next = PeriodicKF_estimate(b,U,Zg,lnr_sys_for_prd,lnr_sys_for_update,Periodic_Kalman_gain)
            disp('There is no mathmatical basis for the periodic KF if your system is not linear. So, do not use this. Use LKF and provide the periodic linear system as its inputs. The problem with periodic KF is that the estimation covariance can become unsymmetric easily.')
            if nargin ~= 6
                error('Ali: In Periodic_KF the linearized system for prediction and update and the Kalman Gain have to be provided as the function inputs.')
            end
            b_prd = PKF.prediction(b,U,lnr_sys_for_prd);
            % The update function in StationaryKF is different from the
            % update function in LKF and EKF.
            b_next = PKF.update_with_periodic_gain(b_prd,Zg,lnr_sys_for_update,Periodic_Kalman_gain);
        end
        function b = update_with_periodic_gain(b_prd,Zg,lnr_sys,periodic_Kalman_gain)
            % lnr_sys is the linear or linearized system, Kalman filter is
            % designed for.
            H = lnr_sys.H;
            % R = lnr_sys.R; % since we use the periodic Kalman gain as
            % the input here, we do not need the matrix R.
            Pprd = b_prd.est_cov;
            % I think in following line changing "inv" to "pinv" fixes possible
            % numerical issues
            KG = periodic_Kalman_gain; %KG is the "Kalman Gain"
            
            Xprd = b_prd.est_mean.val;
            innov = ObservationModel_class.compute_innovation(Xprd,Zg);
            Xest_next = Xprd+KG*innov;
            Pest_next = Pprd-KG*H*Pprd;
            
            % Due to the usage of approximate Kalman gain in periodic
            % kalman filter (Actually, we do not compute the gain based on
            % the current covariance. and we compute it based on the
            % periodic pre-computed covariance), thus the estimation covariance can
            % get too unsymmetic.
            if max(max(abs(Pest_next-Pest_next')))<1e-12
                Pest_next = (Pest_next+Pest_next')/2;  %eliminating small unsymmetricity
            else
                % note that you should never remove following error. If you
                % need a symmetric covariance, you must use the "update"
                % function that computes the Kalman gain based on the
                % predicted covariance.
                error('AliFW: Robot is out of the valid linearization domain of the "periodic LQG controller". As a result, estimation covariance is too unsymmetric!!')
            end
            
            b = belief(state(Xest_next),Pest_next);
        end
        function [K_periodic_correct , Pprd_periodic, Pest_periodic] = periodic_gain_and_covariances(lnr_sys_periodic)
            % The "lnr_sys_periodic" is a "1 by T" array of "linear system"
            % objects.
            T = size(lnr_sys_periodic , 2);  % period
            LSS = lnr_sys_periodic;
            % Memory preallocation. % To see the following relation about
            % matrix sizes, you have to look at the "dpre" function
            % documentation.
            A_DPRE_KF = nan(  size(LSS(1).A',1)  ,  size(LSS(1).A',2)  ,  T  );
            B_DPRE_KF = nan(  size(LSS(1).H',1)  ,  size(LSS(1).H',2)  ,  T  );
            Q_DPRE_KF = nan(  size(LSS(1).G*LSS(1).Q*LSS(1).G',1)  ,  size(LSS(1).G*LSS(1).Q*LSS(1).G',2)  ,  T  );
            R_DPRE_KF = nan(  size(LSS(1).M*LSS(1).R*LSS(1).M',1)  ,  size(LSS(1).M*LSS(1).R*LSS(1).M',2)  ,  T  );
            
            for k=1 : T % In this for loop, the system matrices names are changed to the ones that are needed in DPRE solver for Kalman Filtering.
                % time reversing % This is an exteremly tricky part. To do it
                % correctly you must have to write a simple example on the paper
                % with for example T = 5 to see how this time reversing works. You
                % can see the math in latex in dpre function.
                if k==T
                    new_k = T;
                else
                    new_k = T - k;
                end
                % following follows from $$\overline{k}=T-k $$ and $$ E_k = I,~S_k = 0,~X_k = P^-_{\overline{k}+1},~Q_k = G_{\overline{k}}Q_{\overline{k}}G^T_{\overline{k}},~ R_k = M_{\overline{k}}R_{\overline{k}}M_{\overline{k}}^T,~A_k=A^T_{\overline{k}},~B_k=H^T_{\overline{k}} $$
                A_DPRE_KF(:,:,k) = LSS(new_k).A';
                B_DPRE_KF(:,:,k) = LSS(new_k).H';
                Q_DPRE_KF(:,:,k) = LSS(new_k).G*LSS(new_k).Q*LSS(new_k).G';
                R_DPRE_KF(:,:,k) = LSS(new_k).M*LSS(new_k).R*LSS(new_k).M';
            end
            
            [X_DPRE_KF,K_DPRE_KF_wrong] = dpre(A_DPRE_KF,B_DPRE_KF,Q_DPRE_KF,R_DPRE_KF,[],[],1e-6,1e3);  %#ok<NASGU>
            Pprd_periodic = flipdim(X_DPRE_KF,3);  % reverse the time  k = T - k_new
            
            Pest_periodic = nan(size(LSS(1).A,1)  , size(LSS(1).A,1)  , T);
            K_periodic_correct = nan(size( LSS(1).A,1)  , size(LSS(1).H,1) , T );
            for k = 1:T
                %  K_periodic_wrong(:,:,k) = A_DPRE_KF(:,:,k)\(K_DPRE_KF(:,:,k)');  % inv(A_DPRE_KF(:,:,k)) * K_DPRE_KF(:,:,k)'; % Periodic Kalman Gain
                K_periodic_correct(:,:,k) = (Pprd_periodic(:,:,k)*LSS(k).H')/(LSS(k).H*Pprd_periodic(:,:,k)*LSS(k).H'+LSS(k).M*LSS(k).R*LSS(k).M');
                Pest_periodic(:,:,k) = (eye(MotionModel_class.stDim)-K_periodic_correct(:,:,k)*LSS(k).H)*Pprd_periodic(:,:,k);  % Periodic estimation covariance
            end
        end
        function [X,K] = dpre(A,B,Q,R,S,E,tol,maxit)
            %DPRE Discrete-time Periodic Riccati Equation
            %  [X,K]=DPRE(A,B,Q,R,S,E) computes the unique stabilizing solution X{k},
            %  k = 1:P, of the discrete-time periodic Riccati equation
            %
            %   E{k}'X{k}E{k} = A{k}'X{k+1}A{k} - (A{k}'X{k+1}B{k} + S{k})*...
            %                 (B{k}'X{k+1}B{k} + R{k})\(A{k}'X{k+1}B{k} + S{k})' + Q{k}
            %
            %% DPRE Equation:
            % $$ E_k^TX_kE_k = A^T_kX_{k+1}A_k - (A_{k}^TX_{k+1}B_{k} + S_{k})
            % (B_{k}^TX_{k+1}B_{k} + R_{k})^{-1} (A_{k}^TX_{k+1}B_{k} + S_{k})^T + Q_{k} $$
            
            % Do not remove above space. Every latex code has to be followed by an space
            %  When omitted, R, S and E are set to the default values R{k}=I, S{k}=0,
            %  and E{k}=I. Beside the solution X{k}, DPRE also returns the gain matrix
            %
            %   K{k} = (B{k}'X{k+1}B{k} + R{k})\(B{k}'X{k+1}A{k} + S{k}'),
            %
            %% Returned gain matrix equation:
            % $$ K_{k} = (B_{k}^TX_{k+1}B_{k} + R_{k})^{-1}(B_{k}^TX_{k+1}A_{k} + S_{k}^T) $$
            
            % All input matrices have to be multidimensional arrays, like matrix
            %  A(N,N,P) and B(N,R,P). Output matrices are also multidimensional arrays
            %  in the size of X(N,N,P) and K(R,N,P).
            %
            %  [X,K]=DPRE(A,B,Q,R,S,E,TOL) specifies the tolerance of the cyclic qz
            %  method. If tol is [] then DPRE uses the default, 1e-6.
            %
            %  [X,K]=DPRE(A,B,Q,R,S,E,TOL,MAXIT) specifies the maximum number of
            %  iterations. If MAXIT is [] then DPRE uses the default, 1000. Warning is
            %  given when the maximum iterations is reached.
            %
            %  See also DARE.
            
            %  This version uses a cyclic qz method, see references.
            
            %  References:
            %    [1] J.J. Hench and A.J. Laub, Numerical solution of the discrete-time
            %        periodic Riccati equation, IEEE Trans. on automatic control, 1994
            %    [2] Varga, A., On solving discrete-time periodic Riccati equations,
            %        Proc. of 16th IFAC World Congress 2005, Prague, July 2005.
            
            %  Ivo Houtzager
            %
            %  Delft Center of Systems and Control
            %  The Netherlands, 2007
            
            %% Rewriting DPRE Equation, solved in this m-file:
            % $$ E_k^TX_kE_k = A^T_kX_{k+1}A_k - (A_{k}^TX_{k+1}B_{k} + S_{k})
            % (B_{k}^TX_{k+1}B_{k} + R_{k})^{-1} (A_{k}^TX_{k+1}B_{k} + S_{k})^T +
            % Q_{k} $$
            
            %% Rewriting DPRE Equation, solved in this m-file:
            % $$ K_{k} = (B_{k}^TX_{k+1}B_{k} + R_{k})^{-1}(B_{k}^TX_{k+1}A_{k} + S_{k}^T) $$
            
            %% By the notation change for LQR DPRE
            % $$ E_k = I,~S_k = 0,~X_k = S_k,~Q_k = W_x,~ R_k = W_u $$, we get
            
            %% we get Ali's notation for DPRE of LQR, :
            % $$ S_k = A^T_kS_{k+1}A_k - (A_{k}^TS_{k+1}B_{k})
            % (B_{k}^TS_{k+1}B_{k} + W_u)^{-1} (A_{k}^TS_{k+1}B_{k})^T + W_x$$
            
            %% In solving DPRE of LQR, Ali's notation for gain:
            % $$ L_k = (B_{k}^TS_{k+1}B_{k} + W_u)^{-1} B_{k}^TS_{k+1}A_{k} $$
            
            %% By notation change for Kalman Filter DPRE
            % $$\overline{k}=T-k $$ and $$ E_k = I,~S_k = 0,~X_k = P^-_{\overline{k}+1},~Q_k = G_{\overline{k}}Q_{\overline{k}}G^T_{\overline{k}},~ R_k = M_{\overline{k}}R_{\overline{k}}M_{\overline{k}}^T,~A_k=A^T_{\overline{k}},~B_k=H^T_{\overline{k}} $$
            % we get
            
            %% we get Ali's notation for DPRE of Kalman Filter:
            % $$ P^-_{{\overline{k}}+1} = A_{\overline{k}}P^-_{\overline{k}}A_{\overline{k}}^T - (A_{\overline{k}}P^-_{\overline{k}}H_{\overline{k}}^T)
            % (H_{\overline{k}}P^-_{\overline{k}}H_{\overline{k}}^T + M_{\overline{k}}R_{\overline{k}}M_{\overline{k}}^T)^{-1} (A_{\overline{k}}P^-_{{\overline{k}}+1}H_{\overline{k}}^T)^T + G_{\overline{k}}Q_{\overline{k}}G^T_{\overline{k}}$$
            
            %% and for Kalman Filter, we get the gain:
            % $$ K_{\overline{k}}^{computed} = (H_{\overline{k}}P^-_{\overline{k}}H_{\overline{k}}^T+M_{\overline{k}}R_{\overline{k}}M_{\overline{k}}^T)^{-1}(H_{\overline{k}}P^-_{\overline{k}}A^T_{\overline{k}})$$
            
            %% So, to get the Kalman filter in Ali's notation, we have to apply following transformation:
            % $$ K_{\overline{k}}=A^{-1}_{\overline{k}}(K_{\overline{k}}^{computed})^T $$
            
            % assign default values to unspecified parameters
            [m,n,p] = size(A);
            [mb,r,pb] = size(B);
            if (nargin < 8) || isempty(maxit)
                maxit = 1000;
            end
            if (nargin < 6) || isempty(E)
                E = zeros(m,n,p);
                for i = 1:p
                    E(:,:,i) = eye(m,n);
                end
            end
            if (nargin < 5) || isempty(S)
                S = zeros(mb,r,pb);
            end
            if (nargin < 4) || isempty(R)
                R = zeros(r,r,pb);
                for i = 1:pb
                    R(:,:,i) = eye(r);
                end
            end
            if (nargin < 7) || isempty(tol)
                tol = 1e-6;
            end
            
            % check input arguments
            if nargin < 2
                error('DPRE requires at least three input arguments')
            end
            [mq,nq,pq] = size(Q);
            [mr,nr,pr] = size(R);
            [ms,ns,ps] = size(S);
            [me,ne,pe] = size(E);
            if ~isequal(p,pb,pq,pr,ps,pe)
                error('The number of periods must be the same for A, B, Q, R, S and E.')
            end
            if ~isequal(m,me,mq,mb)
                error('The number of rows of matrix A, B, E and Q must be the same size.')
            end
            if ~isequal(n,ne,nq)
                error('The number of columns of matrix A, E and Q must be the same size.')
            end
            if ~isequal(mb,ms)
                error('The number of rows of matrix B and S must be the same size.')
            end
            if ~isequal(r,ns)
                error('The number of columns of matrix B and S must be the same size.')
            end
            if ~isequal(r,nr)
                error('The number of columns of matrix R must be the same size as the number of columns of matrix B.')
            end
            if ~isequal(r,mr)
                error('The number of rows of matrix R must be the same size as the number of columns of matrix B.')
            end
            
            % allocate matrices
            M = zeros(2*n+r,2*n+r,p);
            L = zeros(2*n+r,2*n+r,p);
            V = zeros(2*n+r,2*n+r,p);
            Z = zeros(2*n+r,2*n+r,p);
            Y = zeros(2*n+r,2*n+r,p);
            T = zeros(n+r,n,p);
            
            % build the periodic matrix pairs
            for i = 1:p
                L(:,:,i) = [A(:,:,i) zeros(n) B(:,:,i);
                    -Q(:,:,i) E(:,:,i) -S(:,:,i);
                    S(:,:,i)' zeros(r,n) R(:,:,i)];
                M(:,:,i) = [E(:,:,i) zeros(n,n+r);
                    zeros(n) A(:,:,i)' zeros(n,r);
                    zeros(r,n) -B(:,:,i)' zeros(r)];
                V(:,:,i) = eye(2*n+r);
                Z(:,:,i) = eye(2*n+r);
            end
            
            % cyclic qz decomposition
            k = 1;
            ok = true;
            res = ones(1,p);
            while ok == true && k <= maxit
                for j = 1:p
                    % QR decomposition
                    [Q,R] = qr(M(:,:,j));
                    V(:,:,j) = V(:,:,j)*Q';
                    M(:,:,j) = Q'*M(:,:,j);
                    
                    % RQ decomposition
                    [Q,R] = qr(fliplr((Q'*L(:,:,j))'));
                    Q = fliplr(Q);
                    R = fliplr(flipud(R))';
                    
                    Z(:,:,j) = Z(:,:,j)*Q;
                    Y(:,:,j) = Q;
                    L(:,:,j) = R;
                end
                
                for j = 1:p
                    if j == p
                        M(:,:,p) = M(:,:,p)*Y(:,:,1);
                    else
                        M(:,:,j) = M(:,:,j)*Y(:,:,j+1);
                    end
                    
                    T1 = Z(n+1:2*n+r,1:n,j)/Z(1:n,1:n,j);
                    
                    % calculate residue
                    res(j) = norm(T1 - T(:,:,j));
                    T(:,:,j) = T1;
                end
                
                if all(res <= tol)
                    ok = false;
                end
                
                k = k + 1;
            end
            
            % return warning if k exceeds maxit
            if ok == true
                warning('DPRE:maxIt','Maximum number of iterations exceeded.')
            end
            
            % retrieve K and X matrices
            X = zeros(n,n,p);
            K = zeros(r,n,p);
            for i = 1:p
                X(:,:,i) = T(1:n,1:n,i);
                K(:,:,i) = -T(n+1:n+r,1:n,i);
            end
        end
    end
end