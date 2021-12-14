function writeBboxes(imagesDatastore)

bboxFiles = strrep(imagesDatastore.Files, '_image.jpg', '_bbox.bin');
projMatFiles = strrep(imagesDatastore.Files, '_image.jpg', '_proj.bin');

im = imread(imageDatastore.Files{1});
im_sz = size(im);

bboxes = cell(numel(bboxFiles), 1);
for i = 1:numel(bboxFiles)
    % Get centroid and size of bounding box. This is in 3d coordinates. 
    % We need 2d pixel coordinates for bounding box regression (maybe).
    
    bbox = reshape(read_bin(bboxFiles{i}), 11, []);
    proj = reshape(read_bin(projMatFiles{i}), 4, 3)';
    
    assert(numel(bbox) == 11);
    
    R = rot(bbox(1:3));
    t = reshape(bbox(4:6), [3, 1]);
    sz = bbox(7:9);
    [vert_3D, ~] = get_bbox(-sz / 2, sz / 2);
    vert_3D = R * vert_3D + t;
    vert_2D = proj * [vert_3D; ones(1, size(vert_3D, 2))];
    vert_2D = vert_2D ./ vert_2D(3, :);
    
    x = max(round(min(vert_2D(1, :))), 1);
    y = max(round(min(vert_2D(2, :))), 1);
    
    w = round(max(vert_2D(1, :)) - x);
    if w + x > im_sz(2)
        w = im_sz(2) - x;
    end
    
    h = round(max(vert_2D(2, :)) - y);
    if h + y > im_sz(1)
        h = im_sz(1) - y;
    end
    
    bboxes{i, 1} = [x y w h]; 
end

bboxTable = table(imageDatastore.Files, bboxes, 'VariableNames', {'imagesFile', 'vehicle'});
save('boundingBoxes.mat', 'bboxTable')
end