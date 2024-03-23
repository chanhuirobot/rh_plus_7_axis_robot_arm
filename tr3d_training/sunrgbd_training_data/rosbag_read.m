bag_file_name = 'max_clock_15';

bagReader = ros2bagreader(bag_file_name);

%msgs = readMessages(bagReader);
bagSubset = select(bagReader,"Topic","/camera/depth_registered/points");
PointCloudmsg = readMessages(bagSubset);

colorBagSubset = select(bagReader, "Topic", "/camera/color/image_raw");
colorMsg = readMessages(colorBagSubset);


% 폴더 구조 생성
folderName = strcat('/home/minipin/Downloads/training/Extraction/', bag_file_name);
if ~exist(folderName, 'dir')
    mkdir(folderName);
end

% Create subfolders within the main folder
depthFolder = fullfile(folderName, 'depth');
labelFolder = fullfile(folderName, 'label');
imageFolder = fullfile(folderName, 'image');
mkdir(depthFolder);
mkdir(labelFolder);
mkdir(imageFolder);

radian = deg2rad(27.5);
%cos_theta = cos(radian);
%sin_theta = sin(radian);
%convert_to_Rotation = [1,0,0;0,-sin_theta,-cos_theta;0,cos_theta,-sin_theta];
% convert_to_DepthCoordinate = [1,0,0;0,0,1;0,1,0];
% Point Cloud 메시지 처리
% numel(PointCloudmsg): The number of pointcloud

for i = 1:40
    xyz = rosReadXYZ(PointCloudmsg{i});
    %xyz_transformed = xyz;
    %xyz_pointcloud = pointCloud(xyz_transformed);

    %%%%% Rotation %%%%%
    [model,inlierIndices, outlierIndices] = pcfitplane(pointCloud(xyz), 0.005);

    % 두 법선 벡터 계산
    normal_vector_plane = model.Normal(1:3);
    if normal_vector_plane(3) > 0
        normal_vector_plane = -1 * normal_vector_plane;
    end
    desired_normal_vector = [0, 0, 1];

    % 회전 축 계산
    rotation_axis = cross(normal_vector_plane, desired_normal_vector);
    rotation_axis = rotation_axis / norm(rotation_axis);

    % 두 벡터 사이의 각도 계산
    angle_rad = acos(dot(normal_vector_plane, desired_normal_vector) / (norm(normal_vector_plane) * norm(desired_normal_vector)));

    % Rodrigues 회전 공식을 사용하여 회전 행렬 계산
    K = [0, -rotation_axis(3), rotation_axis(2);
         rotation_axis(3), 0, -rotation_axis(1);
         -rotation_axis(2), rotation_axis(1), 0];
    R = eye(3) + sin(angle_rad) * K + (1 - cos(angle_rad)) * K^2;

    % 포인트 클라우드 회전
    xyz_transform = (R * xyz')';
    xyz_transform(:,1) = xyz_transform(:,1) + 0.039;
    xyz_transform(:,2) = xyz_transform(:,2) + 0.041;
    xyz_transform(:,3) = xyz_transform(:,3) + 0.43;
    xyz_transform_pointcloud = pointCloud(xyz_transform);
    
    % Visualization
    plane1 = select(pointCloud(xyz), inlierIndices);
    figure
    pcshow(plane1)
    title("First Plane")
    %disp(normal_vector_plane)
    %%%%%%%%%%%%%%%%%%%%
    % .pcd 파일 생성
    pcd_fileName = sprintf('point_cloud_%d.pcd', i);
    pcwrite(xyz_transform_pointcloud, fullfile(labelFolder, pcd_fileName));
    
    
    point_rgb = rosReadRGB(PointCloudmsg{i});
    instance = [xyz_transform, point_rgb];
    
    mat_fileName = sprintf('%06d.mat', i);
    save(fullfile(depthFolder, mat_fileName), 'instance');

end



% Color 메시지 처리
for i = 1:40
    % RGB 값 추출
    image_rgb = rosReadImage(colorMsg{i});
   
    % .jpg 파일로 저장
    image_fileName = sprintf('%06d.jpg', i);
    imwrite(image_rgb, fullfile(imageFolder, image_fileName));

end
