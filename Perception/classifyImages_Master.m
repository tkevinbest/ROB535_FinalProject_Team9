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
% featureNetwork = resnet50(); 
% featureNetwork = googlenet(); 
% featureNetwork = resnet101();
featureNetwork = nasnetlarge(); 

desiredImageSize = featureNetwork.Layers(1).InputSize; 
dataAugmenter = imageDataAugmenter('RandXReflection', true, ...
                                   'RandRotation', [-15, 15], ...
                                   'RandScale', [0.25, 1.5], ...
                                   'RandXTranslation', [-10, 10], ...
                                   'RandYTranslation', [-5, 5]);

trainingSet = trainingImages;

resizedTrainingSet = augmentedImageDatastore(desiredImageSize, trainingSet, 'DataAugmentation', dataAugmenter); 

%% Train a classifier from the extracted features using MLP
% Skip the softmax and final classification layer
% featureLayer = 'fc1000';
% featureLayer = 'loss3-classifier';
% featureLayer = 'predictions';
featureLayer = 'global_average_pooling2d_2';
activationsAtFeatureLayer = activations(featureNetwork, resizedTrainingSet, featureLayer, 'OutputAs', 'columns', 'MiniBatchSize',50);
numFeaturesFound = size(activationsAtFeatureLayer,1); 

%% Train a classifier from the extracted features using an MLP
% classifier = fitcecoc(activationsAtFeatureLayer, trainingImages.Labels, 'Learners','linear','Coding','onevsall',ObservationsIn='columns');

classifierLayers = [
    featureInputLayer(numFeaturesFound);
% FC layer 1
fullyConnectedLayer(1000)
% FC layer 2
fullyConnectedLayer(500)
% FC layer 3
fullyConnectedLayer(100)
% FC layer 4
fullyConnectedLayer(3)
% Softmax layer
softmaxLayer
% Classification layer
classificationLayer
];  

idx = randperm(size(activationsAtFeatureLayer,2),500);
XValidation = activationsAtFeatureLayer(:,idx);
activationsAtFeatureLayer(:,idx) = [];
labels = trainingImages.Labels;
YValidation = trainingImages.Labels(idx);
labels(idx) = [];

options = trainingOptions('adam', ...
    'MaxEpochs',100,...
    'ValidationData',{XValidation',YValidation}, ...
    'InitialLearnRate',1e-4, ...
    'L2Regularization',0.0075,'LearnRateSchedule','piecewise',...
    'Verbose',false, ...
    'MiniBatchSize',200, ...
    'Plots','training-progress');

[classifier, trainingInfo] = trainNetwork(activationsAtFeatureLayer', labels, classifierLayers, options);

%% Evaluate
resizeTestingSet = augmentedImageDatastore(desiredImageSize, testingImages); 
testFeatures = activations(featureNetwork, resizeTestingSet, featureLayer, 'OutputAs','columns','ExecutionEnvironment','gpu', 'MiniBatchSize',50); 
testLabels = classify(classifier, testFeatures');
writeOutputFile(testingImages.Files, testLabels); 
