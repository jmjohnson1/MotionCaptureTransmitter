function myslider

% Create figure window
fig = uifigure;
fig.Name = "Marker Positions";

% Manage app layout
gl = uigridlayout(fig,[2 2]);
gl.RowHeight = {50,'1x'};
gl.ColumnWidth = {'fit','1x'};

% Create UI components
data = getData();
ax = uiaxes(gl);
sld = uislider(gl);

% Lay out UI components
% Position label
lbl.Layout.Row = 1;
lbl.Layout.Column = 1;
% Position drop-down
sld.Layout.Row = 1;
sld.Layout.Column = 2;
% Position axes
ax.Layout.Row = 2;
ax.Layout.Column = [1 2];

% Configure UI component appearance
lbl.Text = "time:";

% Data
sld.Limits = [1, length(data.m0)];

pos = [0, 0, 0];

plots.m0plt = plot3(ax, pos(1), pos(2), pos(3), 'r.', MarkerSize=10);
hold(ax, "on");
hold(ax, "off");
ax.XLim = [-100 8000];
ax.YLim = [-1500 2500];
ax.ZLim = [-2500, 1000];






sld.ValueChangingFcn = @(src,event)updatePlot(src,event,data,...
    plots);

end

function data = getData()
    load("mocapData.mat");
    data.time = timeVec;
    data.m0 = marker0;
    data.m1 = marker1;
    data.m2 = marker2;
    data.m3 = marker3;
    data.m4 = marker4;
    data.m5 = marker5;
    data.m6 = marker6;
    data.m7 = marker7;
end

% Program app behavior
function updatePlot(src, event, data, pos)
 idx = round(event.Value);
 plots.m0plt.XData = data.m0(idx, 1);
 plots.m0plt.YData = data.m0(idx, 2);
 plots.m0plt.ZData = data.m0(idx, 3);
 %disp(data.m0(idx,:));
 drawnow;
 refreshdata;
end
