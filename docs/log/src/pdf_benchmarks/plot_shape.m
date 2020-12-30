gauss = makedist('HalfNormal','mu',0,'sigma',1);
x = 0:0.1:10;

pdf1=pdf(gauss,x);
f=figure();
plot(x,pdf1);
hold on;
plot(x,custom(x));

xlabel('x');
ylabel('pdf(x)');
legend("Gaussian","Alternate function");



exportgraphics(f, "pdf_shape.pdf", 'BackgroundColor', 'none', 'ContentType', 'vector');

function y = custom(x)
    len = length(x);
    y   = ones(len)'./(x.^2+1)'*0.8;
end