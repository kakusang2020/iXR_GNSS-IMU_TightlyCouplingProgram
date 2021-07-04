addpath('C:\Users\denshi\Desktop\OnlyINS\0521\0521');
clasdata=csvread('0521clas.csv');
dgnssdata=csvread('0521DGNSS.csv');
gnssvel=csvread('0521velocity.csv');
clasdata(:,1)=round(clasdata(:,1),1);dgnssdata(:,1)=round(dgnssdata(:,1),1);gnssvel(:,1)=round(gnssvel(:,1),1);
stime=max(max(clasdata(1,1),dgnssdata(1,1)),gnssvel(1,1))+0.2;
etime=min(min(clasdata(end,1),dgnssdata(end,1)),gnssvel(end,1));

clasdata_=clasdata;
for i =1:length(clasdata)
    clasdata_(i,2:4)=pos2ecef(clasdata(i,2:4));
end

Q = eye(3)*0.01;
R = eye(3).*5;
A = eye(3);
H = eye(3);

x_hat = [0;0;0];x_result = x;
w_result = [];v_result = [];z_result = [];
Pk_before = eye(3);x = [];%x = [-27846.2405017545;-2843368.66528705;-5690114.86167899];
x_prehat_result = [];x_hat_result = [];
for i=1:length(clasdata_)   
    if clasdata_(i,5) ~= 4
        z=clasdata_(i,2:4);
        dgnssdata_ =dgnssdata(find(round(dgnssdata(:,1),1)==round(clasdata(i,1),1)-0.2),2:4);
        gnssvel_=gnssvel(find(round(gnssvel(:,1),1)==round(clasdata(i,1),1)-0.2),6:8);
        if(~isempty(dgnssdata_) && ~isempty(gnssvel_))
            %% State
            %     x = ((A * x) + w(:,mainloop));
            x = gnssvel_ * 0.2 + dgnssdata_;
            x_result = [x_result; x];
            
            %% Measurement
            %z = H * x + v(:,mainloop);
            z_result = [z_result; z];
            find(round(clasdata(:,1),1)==round(stime,1));
            %% Predict
            Pk_pre = A * Pk_before * A' + Q;
            x_prehat = (A * x');
            x_prehat_result = [x_prehat_result;x_prehat'];
            
            %% Polish
            Kk = Pk_pre * H' * inv(H * Pk_before * H' + R);
            Pk_before = Pk_pre;
            Pk = (eye(3) - Kk * H) * Pk_pre;
            x_hat = x_prehat + Kk * (z' - H * x_prehat);%这里用测量值改正
            x_hat_result = [x_hat_result; x_hat'];
        end
        clasdata_(i,6:8)=x_hat';
    else
        clasdata_(i,6:8)=clasdata_(i,2:4);
    end
end