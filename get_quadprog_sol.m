function delta_U = get_quadprog_sol(A,B,C,PredictionHorizon, ControlHorizon, Reference, x_a)
    
    RR = 2*eye(ControlHorizon,ControlHorizon );
    % constraints to the optimization. 
    A_const = [-1 0; 1 0];
    B_const = [50; 100];
    
    H(1,:)=C*A;
    temp_S(1,:)=C;
    for kk=2:PredictionHorizon 
        H(kk,:)=H(kk-1,:)* A; 
        temp_S(kk,:)=temp_S(kk-1,:)* A; 
    end

    S=zeros(PredictionHorizon,ControlHorizon);
    temp2_S = temp_S*B;
    S(:,1)=temp2_S; % first column of S
    for i=2:ControlHorizon
        S(:,i)=[zeros(i-1,1);temp2_S(1:PredictionHorizon-i+1,1)]; 
    end

    first =  S' * S + RR;
    second = -2 * (Reference - H*x_a)' * S;
    %action = quadprog(first,second);
    action = quadprog(first,second,A_const,B_const);
    delta_U = action(1); % obtain first control action 

end