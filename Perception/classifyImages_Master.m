clearvars -except imageFolder_Training imageFolder_Testing
close all

if ~exist('fasterRCNNResNet50EndToEndVehicleExample.mat','file')
    disp('Downloading pretrained detector (118 MB)...');
    pretrainedURL = 'https://www.mathworks.com/supportfiles/vision/data/fasterRCNNResNet50EndToEndVehicleExample.mat';
    websave('fasterRCNNResNet50EndToEndVehicleExample.mat',pretrainedURL);
end

% Load the images
if(~exist('imageFolder_Testing','var'))
    imageFolder_Training = uigetdir('.','Locate the training image folder on your computer:');
    imageFolder_Testing = uigetdir([imageFolder_Training,'\..'],'Locate the testing image folder on your computer:');
end

warning('off','MATLAB:table:ModifiedAndSavedVarnames');
trainLabels = readtable((fullfile(imageFolder_Training, 'labels.csv')));
trainingImages = imageDatastore(imageFolder_Training,'IncludeSubfolders',true); 
testingImages = imageDatastore(imageFolder_Testing, 'IncludeSubfolders',true); 

if ~exist('boundingBoxes.mat', 'file')
    writeBboxes(trainingImages)
end

bboxData = load('boundingBoxes.mat');
bboxTable = bboxData.bboxTable;
blds = boxLabelDatastore(bboxTable(:, 'vehicle'));

%%

pretrained = load('fasterRCNNResNet50EndToEndVehicleExample.mat');
lgraph = layerGraph(pretrained.detector.Network);

desiredImageSize = lgraph.Layers(1).InputSize; 

trainingCombined = combine(trainingImages, blds);
preprocessedTrainingData = transform(trainingCombined, @(data) preprocessData(data, desiredImageSize));
test = read(preprocessedTrainingData);

%% Create pre-trained FasterRCNN and continue training with provided dataset

options = trainingOptions('sgdm',...
    'MaxEpochs',10,...
    'MiniBatchSize',8,...
    'InitialLearnRate',1e-3,...
    'CheckpointPath',tempdir);

[detector, info] = trainFasterRCNNObjectDetector(preprocessedTrainingData, lgraph, options, ...
        'NegativeOverlapRange',[0 0.3], ...
        'PositiveOverlapRange',[0.6 1]);

save('fasterRCNNResNet50Transferred.mat', detector);
%writeOutputFile(testingImages.Files, testLabels); 

function data = preprocessData(data,targetSize)
% Resize image and bounding boxes to targetSize.
scale = targetSize(1:2)./size(data{1},[1 2]);
data{1} = imresize(data{1},targetSize(1:2));

new_bbox = bboxresize(data{2},scale);
% Fix the bounding box scaling. For some reason it sometimes produces
% invalid boxes.
new_bbox(1) = max(min(new_bbox(1), targetSize(2)), 1);
new_bbox(2) = max(min(new_bbox(2), targetSize(1)), 1);
new_bbox(3) = max(min(new_bbox(3), targetSize(2) - new_bbox(1)), 1);
new_bbox(4) = max(min(new_bbox(4), targetSize(1) - new_bbox(2)), 1);
data{2} = new_bbox;
end
