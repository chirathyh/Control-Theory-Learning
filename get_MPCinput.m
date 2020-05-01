function delta_U = get_MPCinput(A,B,C,PredictionHorizon, ControlHorizon, Reference, x_a)
    % mpc controller when D = 0.
    % No constraints are considered.
    
    % Tunable Parameters. 
    r_q = 10; % Effect on output 
    r_r = 5; % Effect on control Input
    % Note: Penalization can be specified for individual steps as well.

    % Calculate the W & Z matrices. 
    W(1,:)=C*A;
    temp_S(1,:)=C;
    for kk=2:PredictionHorizon 
        W(kk,:)=W(kk-1,:)* A; 
        temp_S(kk,:)=temp_S(kk-1,:)* A; 
    end
    Z=zeros(PredictionHorizon,ControlHorizon);
    temp2_S = temp_S*B;
    Z(:,1)=temp2_S; % first column of S
    for i=2:ControlHorizon
        Z(:,i)=[zeros(i-1,1);temp2_S(1:PredictionHorizon-i+1,1)]; 
    end
    
    %Y = W * x_a + Z * delta_U;
    Q = r_q * eye(PredictionHorizon,PredictionHorizon);
    R = r_r * eye(ControlHorizon,ControlHorizon); % tuning parameter. 
    
    % Optimized control action.
    delta_U = zeros(ControlHorizon,1);
    delta_U = inv(R+Z'*Q*Z)*Z'*Q*(Reference - W*x_a);
    
end
