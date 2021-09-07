function Compar4file(inputArg1,inputArg2,inputArg3,inputArg4)
result1 = [];
result2 = [];
result3 = [];
for i = 1:length(inputArg2)
    index = find(inputArg1(:,1) == inputArg2(i,1));
    if ~isnan(index)
    result1=[result1;inputArg1(index,1:4),inputArg2(i,2:4)];
    else
    end
end

for i = 1:length(inputArg2)
    index = find(inputArg3(:,1) == inputArg2(i,1));
    if ~isnan(index)
    result2=[result2;inputArg3(index,1:4),inputArg2(i,2:4)];
    else
    end
end

for i = 1:length(inputArg2)
    index = find(inputArg4(:,1) == inputArg2(i,1));
    if ~isnan(index)
    result3=[result3;inputArg4(index,1:4),inputArg2(i,2:4)];
    else
    end
end

disp('LC max error is :');
fprintf('%6.4f,%6.4f,%6.4f\n',max(abs(result1(:,2)-result1(:,5))),max(abs(result1(:,3)-result1(:,6))),max(abs(result1(:,4)-result1(:,7))));
% disp('LC mean error is :');
% fprintf('%6.4f,%6.4f,%6.4f\n',mean(result1(:,2)-result1(:,5)),mean(result1(:,3)-result1(:,6)),mean(result1(:,4)-result1(:,7)));
% disp('LC std error is :');
% fprintf('%6.4f,%6.4f,%6.4f\n',std(result1(:,2)-result1(:,5)),std(result1(:,3)-result1(:,6)),std(result1(:,4)-result1(:,7)));
% disp('LC 95% confidence interval is :');
% fprintf('%6.4f,%6.4f,%6.4f\n',1.96*std(result1(:,2)-result1(:,5))/size(result1,1)^0.5,1.96*std(result1(:,3)-result1(:,6))/size(result1,1)^0.5,1.96*std(result1(:,4)-result1(:,7))/size(result1,1)^0.5);

disp('TC max error is :');
fprintf('%6.4f,%6.4f,%6.4f\n',max(abs(result2(:,2)-result2(:,5))),max(abs(result2(:,3)-result2(:,6))),max(abs(result2(:,4)-result2(:,7))));
% disp('TC mean error is :');
% fprintf('%6.4f,%6.4f,%6.4f\n',mean(result2(:,2)-result2(:,5)),mean(result2(:,3)-result2(:,6)),mean(result2(:,4)-result2(:,7)));
% disp('TC std error is :');
% fprintf('%6.4f,%6.4f,%6.4f\n',std(result2(:,2)-result2(:,5)),std(result2(:,3)-result2(:,6)),std(result2(:,4)-result2(:,7)));
% disp('TC 95% confidence interval is :');
% fprintf('%6.4f,%6.4f,%6.4f\n',1.96*std(result2(:,2)-result2(:,5))/size(result2,1)^0.5,1.96*std(result2(:,3)-result2(:,6))/size(result2,1)^0.5,1.96*std(result1(:,4)-result1(:,7))/size(result1,1)^0.5);

disp('RTK max error is :');
fprintf('%6.4f,%6.4f,%6.4f\n',max(abs(result3(:,2)-result3(:,5))),max(abs(result3(:,3)-result3(:,6))),max(abs(result3(:,4)-result3(:,7))));

data_max=[max(result3(:,2)-result3(:,5)),max(result1(:,2)-result1(:,5)),max(result2(:,2)-result2(:,5))];
data_mean=[mean(result3(:,2)-result3(:,5)),mean(result1(:,2)-result1(:,5)),mean(result2(:,2)-result2(:,5))];
disp('mean error in E is (GNSS/LC/TC)');
fprintf('%6.4f,%6.4f,%6.4f\n',data_mean(1),data_mean(2),data_mean(3));
data_std=[std(result3(:,2)-result3(:,5)),std(result1(:,2)-result1(:,5)),std(result2(:,2)-result2(:,5))];
disp('std error in E is(GNSS/LC/TC)');
fprintf('%6.4f,%6.4f,%6.4f\n',data_std(1),data_std(2),data_std(3));
% data_95=[std(result3(:,2)-result3(:,5))/size(result3,1)^0.5,std(result1(:,2)-result1(:,5))/size(result1,1)^0.5,std(result2(:,2)-result2(:,5))/size(result2,1)^0.5];
stor_data1=sort(abs(result3(:,2)-result3(:,5)));
stor_data2=sort(abs(result1(:,2)-result1(:,5)));
stor_data3=sort(abs(result2(:,2)-result2(:,5)));
data_95=[stor_data1(round(length(stor_data1)*0.975,0)),stor_data2(round(length(stor_data2)*0.975,0)),stor_data3(round(length(stor_data3)*0.975,0))];
disp('95 in E is(GNSS/LC/TC)');
fprintf('%6.4f,%6.4f,%6.4f\n',data_95(1),data_95(2),data_95(3));

figure (1)
subplot(1,2,1);
b_n=bar([abs(data_max);abs(data_mean);abs(data_95)]);%abs(data_std);
grid on;
ch = get(b_n,'children');
set(gca,'XTickLabel',{'Maximum','Mean','95-th Pc.ile'})%'St.Dev',
% set(gca,'YScale','log');
legend('GNSS','LC','TC');
ylabel('Value (m)');
title('Error in E-W(m)');
set(gca,'FontSize',20,'LineWidth',2);

% set(b_n,'LineWidth',2);

data_max=[max(result3(:,3)-result3(:,6)),max(result1(:,3)-result1(:,6)),max(result2(:,3)-result2(:,6))];
data_mean=[mean(result3(:,3)-result3(:,6)),mean(result1(:,3)-result1(:,6)),mean(result2(:,3)-result2(:,6))];
disp('mean error in N is:(GNSS/LC/TC)');
fprintf('%6.4f,%6.4f,%6.4f\n',data_mean(1),data_mean(2),data_mean(3));
data_std=[std(result3(:,3)-result3(:,6)),std(result1(:,3)-result1(:,6)),std(result2(:,3)-result2(:,6))];
disp('std error in N is(GNSS/LC/TC)');
fprintf('%6.4f,%6.4f,%6.4f\n',data_std(1),data_std(2),data_std(3));
stor_data1=sort(abs(result3(:,3)-result3(:,6)));
stor_data2=sort(abs(result1(:,3)-result1(:,6)));
stor_data3=sort(abs(result2(:,3)-result2(:,6)));
data_95=[stor_data1(round(length(stor_data1)*0.975,0)),stor_data2(round(length(stor_data2)*0.975,0)),stor_data3(round(length(stor_data3)*0.975,0))];
disp('95% in N is(RTK/LC/TC)');
fprintf('%6.4f,%6.4f,%6.4f\n',data_95(1),data_95(2),data_95(3));
% data_95=[std(result3(:,3)-result3(:,6))/size(result3,1)^0.5,std(result1(:,3)-result1(:,6))/size(result1,1)^0.5,std(result2(:,3)-result2(:,6))/size(result2,1)^0.5];

subplot(1,2,2);
b_n=bar([abs(data_max);abs(data_mean);abs(data_95)]);%;abs(data_std)
grid on;
ch = get(b_n,'children');
set(gca,'XTickLabel',{'Maximum','Mean','95-th Pc.ile'})%,'St.Dev'
% set(gca,'YScale','log');
legend('GNSS','LC','TC');
ylabel('Value (m)');
title('Error in N-S(m)');
set(gca,'FontSize',20,'LineWidth',2);

% [MU,SIGMA,MUCI,SIGMACI] = normfit(result3(:,3)-result3(:,6)) ;

figure (2);
subplot(2,1,1);
plot(result3(:,1),result3(:,2)-result3(:,5),'LineWidth',4);
hold on
plot(result1(:,1),result1(:,2)-result1(:,5),'LineWidth',4);
hold on
plot(result2(:,1),result2(:,2)-result2(:,5),'LineWidth',4);
legend('GNSS error','LC error','TC error');
hold off
xlabel('GPS Time (s)');
ylabel('Error (m)');
title('Error in E-W(m)');
set(gca,'FontSize',20,'LineWidth',2);
set(gca,'FontSize',20);
grid on;

subplot(2,1,2);
plot(result3(:,1),result3(:,3)-result3(:,6),'LineWidth',4);
hold on
plot(result1(:,1),result1(:,3)-result1(:,6),'LineWidth',4);
hold on
plot(result2(:,1),result2(:,3)-result2(:,6),'LineWidth',4);
legend('GNSS error','LC error','TC error');
hold off
xlabel('GPS Time (s)');
ylabel('Error (m)');
set(gca,'FontSize',20);
title('Error in N-S(m)');
set(gca,'FontSize',20,'LineWidth',2);
grid on;
% subplot(3,1,3);
% plot(result1(:,1),result1(:,4)-result1(:,7),'LineWidth',4);
% set(gca,'FontSize',20);
% title('Error in U-D(m)');
% grid on;
end

