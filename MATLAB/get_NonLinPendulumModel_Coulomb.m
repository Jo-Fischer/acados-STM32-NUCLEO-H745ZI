function model = get_NonLinPendulumModel_Coulomb(varargin)
    
    import casadi.*
    
    %% system dimensions
    nx = 4;
    nu = 1;
    param = [];
    
    %% Import paramter values
    Parameters_FurutaPendulum_Coulomb();
    
    %% named symbolc variables
    theta1 = SX.sym('theta1');
    theta1p = SX.sym('theta1p');
    theta2 = SX.sym('theta2');
    theta2p = SX.sym('theta2p');
    u = SX.sym('u');
    
    %% unnamed symbolic variables
    x = vertcat(theta1, theta1p, theta2, theta2p);
    xdot = SX.sym('xdot', nx, 1);
    
                           
    %% Hilfsgrößen
    s2 = sin(theta2);
    c2 = cos(theta2);
    s2_2 = sin(2*theta2);
    J2_s2_2 = hJ2*s2_2;
    m2L1l2 = m2*L1*l2;
    
    %% Massmatrix wie beim alten Nen_reg-Ansatz
    MassMa = [hJ0 + hJ2*s2^2, m2L1l2*c2;
              m2L1l2*c2,  hJ2];
    
    %% RHS
    rhs1 =  (-b1vis - kM^2/RM) * theta1p ...
            -J2_s2_2 * theta1p * theta2p ...
            + m2L1l2*s2 * theta2p^2 ...
            + kM/RM * kSteller * u...
            - b1coul * tanh(k_factor * theta1p);
    
    rhs2 =  -b2vis * theta2p... 
             + 0.5*J2_s2_2 * theta1p^2 ...
             -b2coul * tanh(k_factor * theta2p)...
             -l2*m2*s2 * grav;
    
    rhs = vertcat(rhs1, rhs2);
    
    %% Beschleunigungen lösen
    ddq = MassMa \ rhs;
    
    %% Vollständige xdot
    f_expl_expr = vertcat(theta1p, ddq(1), theta2p, ddq(2));
    
    f_impl_expr = vertcat( ...
        xdot(1) - x(2), ...
        xdot(2) - ddq(1),...
        xdot(3) - x(4), ...
        xdot(4) - ddq(2) );
    
    %f_impl_expr = f_expl_expr - xdot;
    
    % discrete dynamics
    if nargin > 0
        delta_t = varargin{1};
        disc_dyn_expr = x + delta_t * f_expl_expr; % explicit Euler
    else
        disc_dyn_expr = [];
    end
       
    
    % populate
    model = AcadosModel();
    model.x = x;
    model.xdot = xdot;
    model.u = u;
    model.p = param;
    
    model.f_expl_expr = f_expl_expr;
    model.f_impl_expr = f_impl_expr;
    model.disc_dyn_expr = disc_dyn_expr;
    model.name = 'FurutaPendulum';
    
end