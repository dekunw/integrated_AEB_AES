    lr=1.67;
    Vx=15;
    threshold=0.213;
    miu=0.7;
    rmax=9.81*miu/Vx;

    Nx=4;%number of state
    Nu=2;%number of control
    Np=30;%prediction horizon
    Nc=1;%control horizon
    Useq=zeros(Np+1,2);
    Vx=15;
    ks=0;

    %reference
    r=zeros(Nx,1);%state reference
    r(1)=0;
    r(2)=0;
    r(3)=0;
    r(4)=0;    
    ur1=0;
    ur2=0;%control reference
    
    %states
    x=zeros(Nx,1);%state reference
    x(1)=0;
    x(2)=0;
    x(3)=0;
    x(4)=0;   
    
    %control
    u=zeros(Nu,1);%state reference
    u(1)=0;
    u(2)=0;
    
    %get augmented states
    kesi=zeros(Nx+Nu,1);%[X;U]
    kesi(1)=x(1)-r(1);
    kesi(2)=x(2)-r(2);
    kesi(3)=x(3)-r(3);
    kesi(4)=x(4)-r(4);
    kesi(5)=u(1);
    kesi(6)=u(2);
    
    Ts=0.1;%sample time
    
%% Modellng->Discretization->prediction->quadratic programming
    %Modelling&Linearization&Discretization
    steps=Np+1;
    A = zeros(Nx+Nu,Nx+Nu,steps);
    B = zeros(Nx+Nu,Nu,steps);
    Dc = zeros(Nx,1,steps);
    C=[1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0];
    steps=Np+1;
    
    for ct = 1:steps
        if ct>length(Useq)
            uk = Useq(Np,:);
        else
            uk = Useq(ct,:);
        end
        uk=uk';
        [Ad, Bd, ~, ~, DK, delta_x] = Dynamics(Vx,ks,x,uk,Ts);
        A(:,:,ct)=[Ad,Bd;zeros(Nu,Nx),eye(Nu)];        
        B(:,:,ct)=[Bd;ones(Nu,Nu)];
        Dc(:,:,ct)=DK;
        x=x+delta_x;    
    end
    %construct important matrix
    coder.varsize('PHI',[Np*Nx,Nx+Nu]);
    coder.varsize('THETA',[Np*Nx,Nc*Nu]);
    coder.varsize('D',[Np*Nx,1]);
    PHI=[];
    THETA=[];
    D=[];
    product=eye(6);
    for j=1:Np
        product=A(:,:,j)*product;
        PHI=[PHI;C*product];
        D=[D;Dc(:,:,j)];
    end
    for k=1:Nc
        product1=eye(6);
        for j=1:Np
            THETA=[THETA;C*product1*B(:,:,k)];
            product1=A(:,:,j+1)*product1;
        end
    end
    Sigma=zeros(Np,Nc);
    for p=1:Np%define a lower triangular matrix
        for q=1:1:Nc
            if q<=p 
                Sigma(p,q)=1;
            else 
                Sigma(p,q)=0;
            end
        end 
    end 
    UT=kron(ones(Np,1),[u(1);u(2)]);
    Sigma=kron(Sigma,ones(Nu,Nu));
    error=PHI*kesi;
    Q=kron(eye(Np,Np),diag([0 0 100 4]));    
    R=kron(eye(Np,Np),diag([0 4e-5]));    
    P=diag([3e-5 8e-5]);
    rou1=3e3;  
    rou2=5e4;
    H=2*[THETA'*Q*THETA+Sigma'*R*Sigma+P zeros(2,2);0 0 rou1 0;0 0 0 rou2];%是否要乘2？
    f=[2*(error'*Q*THETA+D'*Q*THETA+UT'*R*Sigma),0,0];%是否要转置？
    
    %% constraints
    %inequation
    A_t=zeros(Nc,Nc);
    for p=1:1:Nc%define a lower triangular matrix
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu)); 
    Ut=kron(ones(Nc,1),[0;0]);%control in last control step
    umin=[-1e4; -1e3];
    umax=[1e4; 1e3];
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons=[A_I zeros(Nu*Nc,2);-A_I zeros(Nu*Nc,2)];
    b_cons=[Umax-Ut;-Umin+Ut];
    %add handling envelop constriants
    gamas=[1 -lr 0 0;-1 lr 0 0;0 1 0 0;0 -1 0 0];
    for i=1:4:Np
        Gs=[Vx*threshold;Vx*threshold;rmax;rmax];
        A_cons=[A_cons;[gamas*THETA(i:i+3,:) -ones(4,1) zeros(4,1)]];
        b_cons=[b_cons;[Gs-gamas*PHI(i:i+3,:)*kesi]];
    end
    
    %upper and lower boundary
    delta_umin=[-1e4;-1e3];
    delta_umax=[1e4;1e3];
    M1=1; %related to slack variable
    M2=1; %related to slack variable
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0;0];
    ub=[delta_Umax;M1;M2];  