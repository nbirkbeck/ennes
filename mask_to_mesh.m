function mask_to_mesh(im, obj)
M = ~((im(:,:,1) == 255) .* (im(:, :, 2) == 255) .* im(:, :, 3) == 255);
[I,J] = find(M);
[X,Y]=meshgrid(1:size(M, 1), 1:size(M, 2));
shape = size(M);
tri = delaunay(X(sub2ind(shape([2,1]), J, I)), Y(sub2ind(shape([2,1]), J, I)));
pts=[X(sub2ind(shape([2,1]), J, I)), Y(sub2ind(shape([2,1]), J, I))];
writeobj(obj, pts, tri); 

function writeobj(obj, pts, tri)
f=fopen(obj, 'w');
du = pts(tri(:, 2), :) - pts(tri(:, 1), :);
dv = pts(tri(:, 3), :) - pts(tri(:, 1), :);
size(du)
nz = du(:, 1) .* dv(:, 2) - du(:, 2) .* dv(:, 1);
I = find(nz < 0);
I

tri(I, [2,3]) = tri(I, [3,2]);
fprintf(f, 'v %f %f 1\n', pts')
fprintf(f, 'vt %f %f 1\n', diag(1.0./max(pts, [], 1)) * pts')
fprintf(f, 'f %d/%d %d/%d %d/%d\n', tri(:,[1,1,2,2,3,3])')
fclose(f)
