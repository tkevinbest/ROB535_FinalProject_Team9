clear
close all

% Load the images
imageFolder_Training = uigetdir('.','Locate the training image folder on your computer:');
imageFolder_Testing = uigetdir([imageFolder_Training,'\..'],'Locate the testing image folder on your computer:');
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
minExamples = min(imageCounts.Count); 
trainingImages = splitEachLabel(trainingImages, minExamples, 'randomize'); 

%% Load pretrained network for feature extraction
featureNetwork = resnet50(); 

desiredImageSize = featureNetwork.Layers(1).InputSize; 
resizedTrainingSet = augmentedImageDatastore(desiredImageSize, trainingImages, 'ColorPreprocessing', 'gray2rgb'); 

% Skip the softmax and final classification layer
featureLayer = 'fc1000';
activationsAtFeatureLayer = activations(featureNetwork, resizedTrainingSet, featureLayer, 'MiniBatchSize', 32, 'OutputAs', 'columns','ExecutionEnvironment','gpu');

%% Train a classifier from the extracted features using SVM
classifier = fitcecoc(activationsAtFeatureLayer, trainingImages.Labels, 'Learners','linear','Coding','onevsall',ObservationsIn='columns');


%% Evaluate
resizeTestingSet = augmentedImageDatastore(desiredImageSize, testingImages,'ColorPreprocessing','gray2rgb'); 
testFeatures = activations(featureNetwork, resizeTestingSet, featureLayer, 'MiniBatchSize',32, 'OutputAs','columns','ExecutionEnvironment','gpu'); 
testLabels = predict(classifier, testFeatures, 'ObservationsIn','columns');
writeOutputFile(testingImages.Files, testLabels); 
