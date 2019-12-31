% Create an image, where white is background.
% Normally from: ./highres_boundary  images/little_mario_turtle.png 8 1
% Export the obj, load in blender, setup TC, smooth twice, and run blow_up fro
% /home/birkbeck/svn/human/ik/tests
% mario was created with:
%  good=$(./blowup /tmp/mario.obj /tmp/goomba_blowup.obj --alpha 0.05 --ntime 100)
% And the others were created with:
%  ./blowup /tmp/turtle.obj /tmp/turtle_blowup.obj --alpha 0.025 --ntime 50
%
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
edge_length = max((du(:, 1).^2 + du(:, 2).^2).^0.5, ...
                  (dv(:, 1).^2 + dv(:, 2).^2).^0.5)
J = find(edge_length < 2);
tri = tri(J, :);



npoints = size(pts, 1);
ntri = size(tri, 1);
e = sparse(reshape(tri(:,[2,3,1]), [ntri*3, 1]), reshape(tri(:, [1,2,3]), [ntri * 3, 1]), ...
           ones(size(tri, 1) * 3, 1), npoints, npoints);
non_zero = (e - e');
[I,J] = find(non_zero == 1)
plot(pts(I, 1), pts(I, 2), '.');
npoints
ntri
length(I)
etri = [I, J, J + npoints; I + npoints, I, J + npoints];
etri_uv = [I, J, J; I, I, J];
size(etri)

fprintf(f, 'v %f %f 2\n', pts')
fprintf(f, 'v %f %f -2\n', pts')
fprintf(f, 'vt %f %f 1\n', diag(1.0./max(pts, [], 1)) * pts')
fprintf(f, 'f %d/%d %d/%d %d/%d\n', tri(:,[1,1,2,2,3,3])')
fprintf(f, 'f %d/%d %d/%d %d/%d\n', ...
        [tri(:, 1) + npoints, tri(:,1), ...
         tri(:, 3) + npoints, tri(:,3), ...
         tri(:, 2) + npoints, tri(:,2)]');
fprintf(f, 'f %d/%d %d/%d %d/%d\n', ...
        [etri(:, 1), etri_uv(:, 1), ...
         etri(:, 2), etri_uv(:, 2), ...
         etri(:, 3), etri_uv(:, 3)]');
fclose(f)
