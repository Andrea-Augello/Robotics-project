theta = linspace(0,3.14,11);
rho = [3 3.5 3.25 2 2.2 5.99 5.99 5.99 5 4.5 4];
f = figure();
polarplot(theta, rho);
hold on
polarscatter([0 53.97/180*3.14 73.93/180*3.14 3.14],[3 2 2.2 4]);

exportgraphics(f, "bug.pdf", 'BackgroundColor', 'none', 'ContentType', 'vector');
