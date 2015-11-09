function ShowAllFig(f)
%% ShowAllFig Shows all Figures at the Screen
%
% e.g.:
%  f1 = figure('Name', 'A');
%  f2 = figure('Name', 'B');
%  f3 = figure('Name', 'C');
%  f4 = figure('Name', 'D');
%  f = [f1, f2; f3, f4];
%  ShowAllFig(f);
%
% by michael@michael-heuer.net
%

%%

kit = java.awt.Toolkit.getDefaultToolkit();
screenSize = kit.getScreenSize();
instance = com.mathworks.mde.desk.MLDesktop.getInstance();
insets = kit.getScreenInsets(instance.getMainFrame().getGraphicsConfiguration());

%%
x = insets.left;
y = insets.bottom;
w = screenSize.width - insets.left - insets.right;
h = screenSize.height - insets.bottom - insets.top;

[yi, xi] = size(f);
for a = 1:xi
    for b = 1:yi
        p = [1+x+(a-1)*floor(w/xi),1+y+(yi-b)*floor(h/yi),ceil(w/xi),ceil(h/yi)];
        set(f(b,a), 'units','pixel','outerposition', p)
    end
end

% bring to foreground
for ii = 1:numel(f)
figure(f(ii));
end