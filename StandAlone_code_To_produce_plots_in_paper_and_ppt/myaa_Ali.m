function myaa_Ali(filename)

if nargin == 0
    filename = 'untitled_image';
end

screen_DPI = get(0,'ScreenPixelsPerInch');
tempfile_png = [filename,'.png'];
tempfile_pdf = [filename,'.pdf'];
print(gcf,['-r',num2str(screen_DPI*4)], '-dpng', tempfile_png);
% print(gcf,['-r',num2str(screen_DPI*4)], '-dpdf',tempfile_pdf,'-zbuffer','-r600');  % It seems that in most cases
% generating a png and then converting it to a pdf results in a better
% figure than generating a pdf directly.

end