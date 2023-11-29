% Cell has been visited 
% Cell is solid 
function plotMap(maptable)

map


end


function sandbox()
%%

objset = [
    "purple_zombie" "green_zombie" "aqua_zombie" "blue_zombie", ...
    "pink_berry" "orange_berry" "red_berry" "yellow_berry",...
    "wall"
    ]';

colors = colororder;

maxy  = 10;

nobjs = 10;

testmap   = table();
testmap.x = randi(maxy*2, nobjs,1)-maxy;
testmap.y = randi(maxy*2, nobjs,1)-maxy;
testmap.value = objset(randi(numel(objset),nobjs,1));

minmax = @(x) max(abs(max(x(:))), abs(min(x(:)) ));
mxx = minmax(testmap{:,["x" , "y"]});
mgrid = (-mxx:mxx);
nmx   = numel(mgrid);



[gridx,gridy] = deal(repmat(getgrid(mxx),1,2));

cmap = [1 1 1 ; 0 0 0 ; 0.2627    0.6039    0.5255];


getind  = @(x) find(ismember(mgrid , x));


% xfix = repmat([m])
current.x = 0;
current.y = 0;
current.memory = zeros(nmx);

[r , c] = ind2sub([1 1]*(nmx) , [91 111 131 151 150]);

current.memory(getind([0 0 0]) , getind([ 0 1 2])) = 1;

curx = getind(current.x);
cury = getind(current.y);

current.memory = double(logical(current.memory));

current.memory(curx,cury) = 2;

imagesc(mgrid , mgrid  , current.memory ,"AlphaData",0.5)
colormap(cmap);
hold on
scatter(current.x , current.y , 30, 'k', 'filled')

gridlen = repmat([-1 1]*(mxx+.5),size(gridx,1),1);

plot([gridx ;gridlen]', [gridlen ;gridy]','k' , linewidth=3)

vals  = unique(testmap.value);
nvals = numel(vals);

for i = 1:nvals

    c = colors(objset == vals(i),:);

    idx = ismember(testmap.value , vals(i));
    x = testmap.x(idx);
    y = testmap.y(idx);

    scatter(x, y , 300 , c , 'filled','AlphaData',.5 );

end


end

function grid = getgrid(mxx)

    e = mxx+1;
    grid = (-e:e)+.5;
    grid = [grid -flip(grid)]';

end







