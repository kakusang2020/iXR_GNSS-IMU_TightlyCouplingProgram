%clear
% GNSSTCData(:,3) = round(GNSSTCData(:,3),1);
% inon(:,3) = round(inon(:,3),1);
% 
% for i = 1:length(GNSSTCData)
%     i
%     if GNSSTCData(i,3) ~= 0
%         data = find(inon(:,3) == GNSSTCData(i,3));
%         %     for j = 1:length(data)
%         %         if inon(data + j -1,3:4) == GNSSTCData(i,3:4)
%         %             GNSSTCData(i,17) = inon(j,6);
%         %         end
%         %     end
%         index = find(inon(data,4) == GNSSTCData(i,4));
%         if ~isnan(index)
%         GNSSTCData(i,17) = inon(index,6);
%         end
%     end
% end
GNSSTCData_ = [];
for i = 1:length(GNSSTCData)
    if GNSSTCData(i,17) ~= 0
        GNSSTCData_ = [GNSSTCData_;GNSSTCData(i,:)];
    end
end