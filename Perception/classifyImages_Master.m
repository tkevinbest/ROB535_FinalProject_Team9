clearvars -except imageFolder_Training imageFolder_Testing
close all

% Load the images
if(~exist('imageFolder_Testing','var'))
    imageFolder_Training = uigetdir('.','Locate the training image folder on your computer:');
    imageFolder_Testing = uigetdir([imageFolder_Training,'\..'],'Locate the testing image folder on your computer:');
end

warning('off','MATLAB:table:ModifiedAndSavedVarnames');
trainLabels = readtable((fullfile(imageFolder_Training, 'labels.csv')));
trainingImages = imageDatastore(imageFolder_Training,'IncludeSubfolders',true, 'Labels',categorical(trainLabels.label)); 
testingImages = imageDatastore(imageFolder_Testing, 'IncludeSubfolders',true); 

bboxFiles = strrep(trainingImages.Files, '_image.jpg', '_bbox.bin');
projMatFiles = strrep(trainingImages.Files, '_image.jpg', '_proj.bin');

bboxes = cell(numel(bboxFiles), 1);
for i = 1:numel(bboxFiles)
    % Get centroid and size of bounding box. This is in 3d coordinates. 
    % We need 2d pixel coordinates for bounding box regression (maybe).
    bbox = reshape(read_bin(bboxFiles{i}), 11, []);
    proj = reshape(read_bin(projMatFiles{i}), 4, 3);
    
    assert(numel(bbox) == 11);
    
    R = rot(bbox(1:3));
    t = reshape(bbox(4:6), [3, 1]);
    sz = bbox(7:9);
    [vert_3D, edges] = get_bbox(-sz / 2, sz / 2);
    vert_3D = R * vert_3D + t;
    vert_2D = proj' * [vert_3D; ones(1, size(vert_3D, 2))];
    vert_2D = vert_2D ./ vert_2D(3, :);
    
    x = round(min(vert_2D(1, :)));
    y = round(min(vert_2D(2, :)));
    w = round(max(vert_2D(1, :)) - x);
    h = round(max(vert_2D(2, :)) - y);
    
    bboxes{i, 1} = [x y w h];
end

bboxTable = table(bboxes);

trainingBboxes = boxLabelDatastore(bboxTable);

trainingCombined = combine(trainingImages, trainingBboxes);
test = read(trainingCombined);

%% Examples to make sure things worked
% Make a subplot of the first 3 types of images
showImages = false; 
if showImages
    otherType = find(trainingImages.Labels == 0); 
    cars = find(trainingImages.Labels == 1); 
    motorcycles = find(trainingImages.Labels == 2); 
    index2show = 10;

    figure(1)
    imshow(readimage(trainingImages, otherType(index2show)))
    figure(2)
    imshow(readimage(trainingImages, cars(index2show)))
    figure(3)
    imshow(readimage(trainingImages, motorcycles(index2show)))
end


%% Preprocess data - balance if desired. May want to remove this because it tosses out a bunch of data
imageCounts = countEachLabel(trainingImages);
minExamples = median(imageCounts.Count); 
% trainingImages = splitEachLabel(trainingImages, minExamples, 'randomize'); 

%% Load pretrained network for feature extraction
featureNetwork = resnet50(); 

lgraph = layerGraph(featureNetwork);

% Remove the last 3 layers. 
layersToRemove = {
    'fc1000'
    'fc1000_softmax'
    'ClassificationLayer_fc1000'
    };
lgraph = removeLayers(lgraph, layersToRemove);

% Specify the number of classes the network should classify.
numClasses = 3;
numClassesPlusBackground = numClasses + 1;

% Define new classification layers.
newLayers = [
    fullyConnectedLayer(numClassesPlusBackground, 'Name', 'rcnnFC')
    softmaxLayer('Name', 'rcnnSoftmax')
    classificationLayer('Name', 'rcnnClassification')
    ];

% Add new object classification layers.
lgraph = addLayers(lgraph, newLayers);

% Connect the new layers to the network. 
lgraph = connectLayers(lgraph, 'avg_pool', 'rcnnFC');

% Define the number of outputs of the fully connected layer.
numOutputs = 4 * numClasses;

% Create the box regression layers.
boxRegressionLayers = [
    fullyConnectedLayer(numOutputs,'Name','rcnnBoxFC')
    rcnnBoxRegressionLayer('Name','rcnnBoxDeltas')
    ];

% Add the layers to the network.
lgraph = addLayers(lgraph, boxRegressionLayers);

% Connect the regression layers to the layer named 'avg_pool'.
lgraph = connectLayers(lgraph,'avg_pool','rcnnBoxFC');

% Select a feature extraction layer.
featureExtractionLayer = 'activation_40_relu';

% Disconnect the layers attached to the selected feature extraction layer.
lgraph = disconnectLayers(lgraph, featureExtractionLayer,'res5a_branch2a');
lgraph = disconnectLayers(lgraph, featureExtractionLayer,'res5a_branch1');

% Add ROI max pooling layer.
outputSize = [14 14];
roiPool = roiMaxPooling2dLayer(outputSize,'Name','roiPool');
lgraph = addLayers(lgraph, roiPool);

% Connect feature extraction layer to ROI max pooling layer.
lgraph = connectLayers(lgraph, featureExtractionLayer,'roiPool/in');

% Connect the output of ROI max pool to the disconnected layers from above.
lgraph = connectLayers(lgraph, 'roiPool','res5a_branch2a');
lgraph = connectLayers(lgraph, 'roiPool','res5a_branch1');

% Define anchor boxes. TODO: Compute better anchor box sizes
% use estimateAnchorBoxes(boxLabelDatastore, numAnchors)
anchorBoxes = [
    16 16
    32 16
    16 32
    ];

% Create the region proposal layer.
proposalLayer = regionProposalLayer(anchorBoxes,'Name','regionProposal');

lgraph = addLayers(lgraph, proposalLayer);

% Number of anchor boxes.
numAnchors = size(anchorBoxes,1);

% Number of feature maps in coming out of the feature extraction layer. 
numFilters = 1024;

rpnLayers = [
    convolution2dLayer(3, numFilters,'padding',[1 1],'Name','rpnConv3x3')
    reluLayer('Name','rpnRelu')
    ];

lgraph = addLayers(lgraph, rpnLayers);

% Connect to RPN to feature extraction layer.
lgraph = connectLayers(lgraph, featureExtractionLayer, 'rpnConv3x3');

% Add RPN classification layers.
rpnClsLayers = [
    convolution2dLayer(1, numAnchors*2,'Name', 'rpnConv1x1ClsScores')
    rpnSoftmaxLayer('Name', 'rpnSoftmax')
    rpnClassificationLayer('Name','rpnClassification')
    ];
lgraph = addLayers(lgraph, rpnClsLayers);

% Connect the classification layers to the RPN network.
lgraph = connectLayers(lgraph, 'rpnRelu', 'rpnConv1x1ClsScores');

% Add RPN regression layers.
rpnRegLayers = [
    convolution2dLayer(1, numAnchors*4, 'Name', 'rpnConv1x1BoxDeltas')
    rcnnBoxRegressionLayer('Name', 'rpnBoxDeltas');
    ];

lgraph = addLayers(lgraph, rpnRegLayers);

% Connect the regression layers to the RPN network.
lgraph = connectLayers(lgraph, 'rpnRelu', 'rpnConv1x1BoxDeltas');

% Connect region proposal network.
lgraph = connectLayers(lgraph, 'rpnConv1x1ClsScores', 'regionProposal/scores');
lgraph = connectLayers(lgraph, 'rpnConv1x1BoxDeltas', 'regionProposal/boxDeltas');

% Connect region proposal layer to roi pooling.
lgraph = connectLayers(lgraph, 'regionProposal', 'roiPool/roi');

analyzeNetwork(lgraph);

desiredImageSize = lgraph.Layers(1).InputSize; 
resizedTrainingSet = augmentedImageDatastore(desiredImageSize, trainingImages, 'ColorPreprocessing', 'gray2rgb'); 

% Skip the softmax and final classification layer
featureLayer = 'fc1000';
activationsAtFeatureLayer = activations(lgraph, resizedTrainingSet, featureLayer, 'OutputAs', 'columns');
numFeaturesFound = size(activationsAtFeatureLayer,1); 

%% Train a classifier from the extracted features using SVM
% classifier = fitcecoc(activationsAtFeatureLayer, trainingImages.Labels, 'Learners','linear','Coding','onevsall',ObservationsIn='columns');

classifierLayers = [
    featureInputLayer(numFeaturesFound);
% FC layer 1
fullyConnectedLayer(1000)
% FC layer 2
% fullyConnectedLayer(1000)
% FC layer 3
fullyConnectedLayer(50)
% FC layer 4
fullyConnectedLayer(3)
% Softmax layer
softmaxLayer
% Classification layer
classificationLayer
];  

options = trainingOptions('adam', ...
    'MaxEpochs',20,...
    'InitialLearnRate',1e-4, ...
    'L2Regularization',0.0005,'LearnRateSchedule','piecewise',...
    'Verbose',false, ...
    'MiniBatchSize',500, ...
    'Plots','training-progress');

[classifier, trainingInfo] = trainFasterRCNNOBjectDetector(trainingImages, lgraph ,options);

%% Evaluate
resizeTestingSet = augmentedImageDatastore(desiredImageSize, testingImages,'ColorPreprocessing','gray2rgb'); 
testFeatures = activations(lgraph, resizeTestingSet, featureLayer, 'OutputAs','columns'); 
testLabels = classify(classifier, testFeatures');
writeOutputFile(testingImages.Files, testLabels); 
