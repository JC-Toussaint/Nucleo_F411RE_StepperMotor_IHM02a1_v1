function circle (x,y,radius, rgb, varargin)
% dessine un cercle de couleur rgb=[r, g, b]
rectangle('position',[x-radius, y-radius, 2*radius, 2*radius], ...
    'FaceColor', [1 1 1], 'EdgeColor', rgb, ...
    'curvature',[1 1],varargin{:});
end
