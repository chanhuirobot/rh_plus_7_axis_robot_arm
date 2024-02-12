clear all;
load('./traintestSUNRGBD/allsplit.mat')
mkdir('/n/fs/modelnet/SUN3DV2/prepareGT/Metadata_v2/');

allPath = [alltest,alltrain];
cnt2d = 1;
cnt3d = 1;
for i =1:length(allPath)
    thispath = allPath{i};
    fprintf('%.2f get 3d gt : %s\n',i/length(allPath)*100,thispath)
    data = readframeSUNRGBD_2d3d(thispath);
    % get better 2D 
    for j =1:length(data.groundtruth3DBB)
        groundtruth(cnt3d) = data.groundtruth3DBB(j);
        cnt3d = cnt3d+1;
    end

    for j =1:length(data.groundtruth2DBB)
        groundtruth_2d(cnt2d) = data.groundtruth2DBB(j);
        cnt2d = cnt2d+1;
    end
    SUNRGBDMeta(i) = data;
    
    
    if 0%mod(i,100)==0
       % draw 2D 
       figure,
       imshow(data.rgbpath);
       hold on; 
       for kk =1:length(data.groundtruth3DBB)
        rectangle('Position', [data.groundtruth3DBB(kk).gtBb2D(1) data.groundtruth3DBB(kk).gtBb2D(2) data.groundtruth3DBB(kk).gtBb2D(3) data.groundtruth3DBB(kk).gtBb2D(4)],'edgecolor','y');
        text(data.groundtruth3DBB(kk).gtBb2D(1),data.groundtruth3DBB(kk).gtBb2D(2),data.groundtruth3DBB(kk).classname,'BackgroundColor','y')
       end
       % draw 3D 
       figure,
       vis_point_cloud(points3d,rgb)
       hold on;
       for kk =1:length(data.groundtruth3DBB)
           vis_cube(data.groundtruth3DBB(kk),'r')
       end
       % draw room
       hold on;
       gtCorner3D = data.gtCorner3D;
       numCorners = size(gtCorner3D,2)/2
       patch(gtCorner3D(1,1:numCorners),gtCorner3D(2,1:numCorners),gtCorner3D(3,1:numCorners),'g');
       patch(gtCorner3D(1,numCorners+1:end),gtCorner3D(2,numCorners+1:end),gtCorner3D(3,numCorners+1:end),'g');
       
       for i=1:numCorners-1
           patch(gtCorner3D(1,[i i+1 numCorners+i+1 numCorners+i]),gtCorner3D(2,[i i+1 numCorners+i+1 numCorners+i]),gtCorner3D(3,[i i+1 numCorners+i+1 numCorners+i]),'g');
           hold on
       end
       
       patch(gtCorner3D(1,[numCorners 1  numCorners+1 numCorners*2]),gtCorner3D(2,[numCorners 1  numCorners+1 numCorners*2]),gtCorner3D(3,[numCorners 1  numCorners+1 numCorners*2]),'g');
       axis equal
       alpha(0.6);
        
       pause; 
    end
end
save('/n/fs/modelnet/SUN3DV2/prepareGT/Metadata_v2/SUNRGBDMeta.mat','SUNRGBDMeta');
save('/n/fs/modelnet/SUN3DV2/prepareGT/Metadata_v2/groundtruth.mat','groundtruth');
save('/n/fs/modelnet/SUN3DV2/prepareGT/Metadata_v2/groundtruth_2d.mat','groundtruth_2d');


%{
load('/n/fs/modelnet/SUN3DV2/prepareGT/Metadata_v2/SUNRGBDMeta.mat','SUNRGBDMeta');
for i = 1:length()
     for j =1:length(data.groundtruth2DBB)
         data.groundtruth2DBB(1).classname = updatename(data.groundtruth2DBB(1).classname);
     end
     for j =1:length(data.groundtruth2DBB)
         data.groundtruth2DBB(1).classname = updatename(data.groundtruth2DBB(1).classname);
     end
     save([thispath '/ data.mat'],'data');
end
%}
