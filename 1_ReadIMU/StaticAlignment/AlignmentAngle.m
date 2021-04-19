
clc,clear
data = csvread('gyro_wheelSpeed.csv');
Eulerk_1=[-1.91299042372587,0.000181803436748563,3.14140917414754] .* 180/pi ;
%q_bk_12nk_1=RotationToQuater(Eulerk_1);
q_bk_12nk_1=EulerToQuater(Eulerk_1);
C_bk_12nk_1=EulerToDCM(Eulerk_1);
BLHk_1=[35.667573968/180*pi;139.791073776/180*pi;39.4716];
e_2=0.00669437999013;
a=6378137;
b=6356752.3141;
f=1/298.257223563;
gamma_a=9.7803267715;
gamma_b=9.8321863685;
we=7.292115*10^-5;
GM=3.986005*10^14;
m=we*we*a*a*b/GM;
RMk_1=a*(1-e_2)/((1-e_2*sin(BLHk_1(1,1))^2))^1.5;
RNk_1=a/(sqrt(1-e_2*sin(BLHk_1(1,1))^2));
delta_v_k_1=[0.0;0.0;0.0];
vnk_1=[0.0;0.0;0.0];%NED
phi_k=[0.0;0.0;0.0];
FinalAnswer=[];
i = 0;
while(~feof(fid))
    i = i + 1;
    x=data(6400 + i,:);
    if(size(x)==0)
        break;
    else if(i <=6400)
        delta_theta_k_1=[x(2,1);x(3,1);x(4,1)] ./ 0.02;
        delta_v_k_1=[x(5,1);x(6,1);x(7,1)] ./ 0.02;%开始没把这速度读进来
        tk_1=x(1,1);
    else if(i > 6400)
        %姿态更新
        y=fread(Answer,10,'double');
        tk=x(1,1);
        delta_theta_k=[x(2,1);x(3,1);x(4,1)];
        delta_v_k=[x(5,1);x(6,1);x(7,1)];
        phi_k=delta_theta_k+1/12*cross(delta_theta_k_1,delta_theta_k);
        q_bk2bk_1=RotationToQuater(phi_k);
        wie2nk_1=[we*cos(BLHk_1(1,1));0;-we*sin(BLHk_1(1,1))];
        wen2nk_1=[vnk_1(2,1)/(RNk_1+BLHk_1(3,1));-vnk_1(1,1)/(RMk_1+BLHk_1(3,1));-vnk_1(2,1)*tan(BLHk_1(1,1))/(RNk_1+BLHk_1(3,1))];
        kxi_k=(wie2nk_1+wen2nk_1)*(tk-tk_1);
        %q_nk_12nk=RotationToQuater(kxi_k);
        q_nk_12nk=kxiToQuater(kxi_k);
        q_bk2nk=QuaterMulti3(q_nk_12nk,q_bk_12nk_1,q_bk2bk_1);
        q_bk2nk=NormQuater(q_bk2nk);
        DCM=QuaterToDCM(q_bk2nk);
        Eulerk=DCMToEuler(DCM);
        %速度更新
        %gamma=(a*gamma_a*(cos(BLHk_1(1,1))^2)+b*gamma_b*(sin(BLHk_1(1,1)))^2)/(sqrt(a*a*(cos(BLHk_1(1,1))^2)+b*b*(sin(BLHk_1(1,1))^2)));
        %gl=gamma*(1-2/a*(1+f+m-2*f*sin(BLHk_1(1,1))^2)*BLHk_1(3,1)+3*BLHk_1(3,1)^2/(a^2));
        grav=[9.7803267715,0.0052790414,0.0000232718,-0.000003087691089,0.000000004397731,0.000000000000721];
        sinB=sin(BLHk_1(1,1));
        sinB2=sinB*sinB;
        sinB4=sinB2*sinB2;
        gl=grav(1,1)*(1.0 + grav(1,2)*sinB2+ grav(1,3)*sinB4) + (grav(1,4) + grav(1,5)*sinB2)*BLHk_1(3,1) + grav(1,6)*BLHk_1(3,1)*BLHk_1(3,1);
        gl2n=[0;0;gl];
        deltavg2nk=(gl2n-cross(2*wie2nk_1+wen2nk_1,vnk_1))*(tk-tk_1);
        deltavfbk_1=delta_v_k+0.5*cross(delta_theta_k,delta_v_k)+(cross(delta_theta_k_1,delta_v_k)+cross(delta_v_k_1,delta_theta_k))/12;
        deltavf2nk=(eye(3)-0.5*VectorAntiMatrix(kxi_k))*C_bk_12nk_1*deltavfbk_1;
        vnk=vnk_1+deltavf2nk+deltavg2nk;
	%位置更新
        BLHk=BLHk_1;
        BLHk(3,1)=BLHk_1(3,1)-1/2*(vnk_1(3,1)+vnk(3,1))*(tk-tk_1);
        h_ave=0.5*(BLHk_1(3,1)+BLHk(3,1));
        BLHk(1,1)=BLHk_1(1,1)+0.5*(vnk_1(1,1)+vnk(1,1))/(RMk_1+h_ave)*(tk-tk_1);
        phi_ave=0.5*(BLHk_1(1,1)+BLHk(1,1));
        BLHk(2,1)=BLHk_1(2,1)+0.5*(vnk_1(2,1)+vnk(2,1))/((RNk_1+h_ave)*cos(phi_ave))*(tk-tk_1);
        FinalAnswer=[x(1,1),BLHk(1,1)*180/pi,BLHk(2,1)*180/pi,BLHk(3,1),vnk',Eulerk'*180/pi]';
        fprintf(FidResult2,'%20.15g\t',FinalAnswer);
        fprintf(FidResult2,'\n');
        FinalAnswer=y-FinalAnswer;
        FinalAnswer(1,1)=x(1,1);
       	%fprintf(FidResult,'%9.5f ','%9.8f ','%9.8f ','%9.8f ','%9.8f ','%9.8f ','%9.8f ','%9.8f ','%9.8f ','%9.8f ',x(1,1),BLHk(1,1)*180/pi,BLHk(2,1)*180/pi,BLHk(3,1),vnk(1,1),vnk(2,1),vnk(3,1),Eulerk(1,1)*180/pi,Eulerk(2,1)*180/pi,Eulerk(3,1)*180/pi);
        fprintf(FidResult,'%20.15g\t',FinalAnswer);
        fprintf(FidResult,'\n');
        %所有k_1时刻的量都要更新到k
        tk_1=tk;
        delta_theta_k_1=delta_theta_k;
        q_bk_12nk_1=q_bk2nk;
        Eulerk_1=Eulerk;
        BLHk_1=BLHk;
        vnk_1=vnk;
        C_bk_12nk_1=EulerToDCM(Eulerk_1);%刚开始没更新这个
        delta_v_k_1=delta_v_k;
        RMk_1=a*(1-e_2)/((1-e_2*sin(BLHk(1,1))^2))^1.5;
        RNk_1=a/(sqrt(1-e_2*sin(BLHk(1,1))^2));
        end
        end
    end
end