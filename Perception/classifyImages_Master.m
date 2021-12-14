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
featureNetwork = googlenet(); 
% featureNetwork = resnet101();

desiredImageSize = featureNetwork.Layers(1).InputSize; 
dataAugmenter = imageDataAugmenter('RandXReflection', true, ...
                                   'RandRotation', [-5, 5], ...
                                   'RandScale', [0.75, 1.5], ...
                                   'RandXTranslation', [-10, 10], ...
                                   'RandYTranslation', [-5, 5]);

trainingSet = trainingImages;
%[trainingSet, validationSet] = splitEachLabel(trainingImages, 0.9);

resizedTrainingSet = augmentedImageDatastore(desiredImageSize, trainingSet, 'DataAugmentation', dataAugmenter); 
%resizedValidationSet = augmentedImageDatastore(desiredImageSize, validationSet); 
  
%analyzeNetwork(featureNetwork);

% Skip the softmax and final classification layer
% featureLayer = 'fc1000';
featureLayer = 'loss3-classifier';
activationsAtFeatureLayer = activations(featureNetwork, resizedTrainingSet, featureLayer, 'OutputAs', 'columns');
numFeaturesFound = size(activationsAtFeatureLayer,1); 

%validationActivations = activations(featureNetwork, resizedValidationSet, featureLayer, 'OutputAs', 'columns');

%% Train a classifier from the extracted features using SVM
% classifier = fitcecoc(activationsAtFeatureLayer, trainingImages.Labels, 'Learners','linear','Coding','onevsall',ObservationsIn='columns');

classifierLayers = [
    featureInputLayer(numFeaturesFound);
% FC layer 1
fullyConnectedLayer(1000)
% FC layer 2
fullyConnectedLayer(500)
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
    'MaxEpochs',25,...
    'InitialLearnRate',1e-4, ...
    'L2Regularization',0.0005, ...
    'LearnRateSchedule','piecewise',...
    'Verbose',false, ...
    'MiniBatchSize',50, ...
    'Plots','training-progress');
 %   'ValidationData', {validationActivations', validationSet.Labels},...


[classifier, trainingInfo] = trainNetwork(activationsAtFeatureLayer', trainingSet.Labels, classifierLayers, options);

%% Evaluate
resizedTestingSet = augmentedImageDatastore(desiredImageSize, testingImages); 
testFeatures = activations(featureNetwork, resizedTestingSet, featureLayer, 'OutputAs','columns'); 
testLabels = classify(classifier, testFeatures');
writeOutputFile(testingImages.Files, testLabels); 
