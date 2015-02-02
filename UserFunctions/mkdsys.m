function sys=mkdsys(m,k,d)
% Gives a mass-spring damper system for force input and position output
% the number of masses is determined from the given array
% of masses!
%
% inputs:
% m is an nx1 array containing the n masses      [m1 m2 m3]
% k is an n+1x1 array containing the n+1 springs [k1 k2 k3 k4]
% d is an n+1x1 array containing the n+1 dampers [d1 d2 d3 d4]
% 
% example for n=3:
%            ____        ____        ____                                                         
% ///|  k1  |    |  k2  |    |  k3  |    | k4   |///                                                                                                                             
% ///|/\/\/\| m1 |/\/\/\| m2 |/\/\/\| m3 |/\/\/\|///                                                           
% ///|  d1  |____|  d2  |____|  d3  |____| d4   |///                                                     
%                                                                     
% outputs:
% sys.ss : state-space system also sys.A,sys.B,sys.C,sys.D
% sys.M  : mass matrix 
% sys.K  : stiffness matrix
% sys.D  : damping matrix


%% general 
n=length(m);


%% ss data
%construct A matrix
Ass=zeros(2*n,2*n);
for rr=1:2*n
    if(isodd(rr))
        Ass(rr,:)=zeros(1,2*n);
        Ass(rr,rr+1)=1;
    end
    if(iseven(rr))
        ii=rr/2;%mass number
        entry=(1/m(ii))*[k(ii) d(ii) -k(ii)-k(ii+1) -d(ii)-d(ii+1) k(ii+1) d(ii+1)];
        for cc=1:2*n     
            nzb=rr-4; zb=zeros(1,nzb);      %number of zeros leading each row
            if(nzb==-2),mid=entry(3:end);   %deal with first row
            else, mid=entry; end;           % "    "     "    "
            zb_mid=[zb mid];                %begin and middle part
            ze=zeros(1,2*n-length(zb_mid)); %number of zeros at the end of the row
            zb_mid_ze=[zb_mid ze];          %construct the row
            Ass(rr,:)=zb_mid_ze(1:2*n);     %take only 2n entrys to cope with last row
        end         
    end   
end
%construct B matrix
Bss=zeros(2*n,n);
for(ii=1:n);
    Bss(2*ii,ii)=1/m(ii);
    B0(2*ii,ii)=1;%used in modal decoupling
end
%construct C matrix
Css=zeros(n,2*n);
for(ii=1:n);
    Css(ii,2*ii-1)=1;
end
% D matrix
Dss=zeros(n,n);


%% Mass, stiffness, damping matrix
M=diag(m);
diagK=[];subdiagK=[];
diagD=[];subdiagD=[];
for(ii=1:n)
    diagK=[diagK k(ii)+k(ii+1)]; 
    diagD=[diagD d(ii)+d(ii+1)]; 
end
for(ii=1:n-1)
    subdiagK=[subdiagK -k(ii+1)];
    subdiagD=[subdiagD -d(ii+1)];
end
K=diag(diagK)+diag(subdiagK,1)+diag(subdiagK,-1);
D=diag(diagD)+diag(subdiagD,1)+diag(subdiagD,-1);

%% modal outputs
[Phi,Omega2]=eig(K,M);
Omega=sqrt(Omega2);
Mm=Phi.'*M*Phi;
Km=Phi.'*K*Phi;
Dm=Phi.'*D*Phi;

Bm=inv(Mm)*Phi.'; %B0 is eye so all masses have inputs
Cm=Phi;           %C0 is eye so all masses have outputs

%when omega_ii is zero, inverse will be infinite
%therefore compute inv(Omega) in special way
diagOmega=diag(Omega);
for ii=1:length(diagOmega)
    if abs(diagOmega(ii))<1E-8;
        diagOmega(ii)=1E-8;
    end
end
iOmega=diag(1./diagOmega);

Z=0.5*inv(Mm)*Dm*iOmega;%damping

%modal form
am=[zeros(n,n) eye(n)   ;
    -Omega2    -2*Z*Omega];
bm=[zeros(n,n);
    Bm];%only force actuators
cm=[Cm zeros(n,n)];%only pos measurements
dm=zeros(n,n);

%modal model
for ii=1:n
    Ami{ii}=[  0             1                  ;
          -Omega2(ii,ii) -2*Z(ii,ii)*Omega(ii,ii)];
    Bmi{ii}=[zeros(1,n);
         Bm(ii,:)];    
    Cmi{ii}=[Cm(:,ii) zeros(n,1)];    
    Gm{ii}=ss(Ami{ii},Bmi{ii},Cmi{ii},zeros(n,n));    
end
    



%% compute outputs
sys.a=Ass;
sys.b=Bss;
sys.c=Css;
sys.d=Dss;
sys.ss=ss(Ass,Bss,Css,Dss);
sys.M=M;
sys.K=K;
sys.D=D;
%modal
sys.am=am;
sys.bm=bm;
sys.cm=cm;
sys.dm=dm;
sys.ssm=ss(am,bm,cm,dm);
sys.Mm=Mm;
sys.Km=Km;
sys.Dm=Dm;
sys.Omega=Omega;
sys.Phi=Phi;
sys.Gm=Gm;
sys.Ami=Ami;
sys.Bmi=Bmi;
sys.Cmi=Cmi;




end%function

function ret=iseven(n)
if(mod(n,2)==0), ret=1; else ret=0; end;
end%private function

function ret=isodd(n)
if(mod(n,2)==0), ret=0; else ret=1; end;
end%private function

