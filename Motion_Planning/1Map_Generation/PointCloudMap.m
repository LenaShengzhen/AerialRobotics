% blocks of Metric Map change to point cloud
function obps = PointCloudMap(blocks, margin)
    obps = [];
    density = 2 * margin;
    for id = 1 : size(blocks, 1)
         leftdownPoint = blocks(id, 1:3) - margin;
         rightUpPoint = blocks(id, 4:6) + margin;
         plane_Z = [];
         for i = leftdownPoint(1,1) : density : rightUpPoint(1,1)
             for j = leftdownPoint(1,2) : density : rightUpPoint(1,2)
                 pz = leftdownPoint(1,3);
                 plane_Z = [plane_Z; [i, j, pz]];
             end
         end
         
         plane_Y = [];
         for i = leftdownPoint(1,1) : density : rightUpPoint(1,1)
             for k = leftdownPoint(1,3) : density : rightUpPoint(1,3)
                 py = leftdownPoint(1,2);
                 plane_Y = [plane_Y; [i, py, k]];
             end
         end
         
         plane_X = [];
         for j = leftdownPoint(1,2) : density : rightUpPoint(1,2)
             for k = leftdownPoint(1,3) : density : rightUpPoint(1,3)
                 px = leftdownPoint(1,1);
                 plane_X = [plane_X; [px, j, k]];
             end
         end
         
         long   = rightUpPoint(1,1) - leftdownPoint(1,1);
         width  = rightUpPoint(1,2) - leftdownPoint(1,2);
         high   = rightUpPoint(1,3) - leftdownPoint(1,3);
         obps   = [obps; plane_X; plane_X + [long 0 0];
                        plane_Y; plane_Y + [0 width 0];
                        plane_Z; plane_Z + [0 0 high]; ];
    end
end

