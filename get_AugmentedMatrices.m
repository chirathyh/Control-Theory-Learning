function [A,B,C] = get_AugmentedMatrices(A_d,B_d, C_d)
    %calculate augmented matrics for the MPC controller.
    [m1,num_states]  =size(C_d); 
    [num_states,num_inputs]=size(B_d); 

    A = eye(num_states+m1,num_states+m1); 
    A(1:num_states,1:num_states)= A_d; 
    A(num_states+1:num_states+m1,1:num_states) = C_d*A_d;

    B = zeros(num_states+m1,num_inputs); 
    B(1:num_states,:)=B_d; 
    B(num_states+1:num_states+m1,:)=C_d*B_d; 

    C=zeros(m1,num_states+m1); 
    C(:,num_states+1:num_states+m1)=eye(m1,m1);
end