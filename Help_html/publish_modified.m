function outputAbsoluteFilename = publish_modified(file,options)
%PUBLISH Publish file containing cells to output file
%   PUBLISH(FILE) evaluates the file one cell at a time in the base
%   workspace.  It saves the code, comments, and results to an HTML file
%   with the same name.  The HTML file is stored, along with other
%   supporting output files, in an "html" subdirectory within the script's
%   directory.
%
%   PUBLISH(FILE,FORMAT) saves the results to the specified format.  FORMAT
%   can be one of the following:
%
%      'html'  - HTML.
%      'doc'   - Microsoft Word (requires Microsoft Word).
%      'pdf'   - PDF.
%      'ppt'   - Microsoft PowerPoint (requires Microsoft PowerPoint).
%      'xml'   - An XML file that can be transformed with XSLT or other 
%                tools.
%      'latex' - LaTeX.  Also sets the default imageFormat to 'epsc2' 
%                unless figureSnapMethod is 'getframe'.
%
%   PUBLISH(FILE,OPTIONS) provides a structure, OPTIONS, that may contain
%   any of the following fields.  If the field is not specified, the first
%   choice in the list is used.
%
%       format: 'html' | 'doc' | 'pdf' | 'ppt' | 'xml' | 'latex'
%       stylesheet: '' | an XSL filename (ignored when format = 'doc', 'pdf', or 'ppt')
%       outputDir: '' (an html subfolder below the file) | full path
%       imageFormat: '' (default based on format)  | any supported by PRINT or IMWRITE, depending on figureSnapMethod
%       figureSnapMethod: 'entireGUIWindow'| 'print' | 'getframe' | 'entireFigureWindow'
%       useNewFigure: true | false
%       maxHeight: [] (unrestricted) | positive integer (pixels)
%       maxWidth: [] (unrestricted) | positive integer (pixels)
%       showCode: true | false
%       evalCode: true | false
%       catchError: true | false
%       createThumbnail: true | false
%       maxOutputLines: Inf | non-negative integer
%       codeToEvaluate: (the file you are publishing) | any valid code
%
%   When publishing to HTML, the default stylesheet stores the original
%   code as an HTML comment, even if "showcode = false".  Use GRABCODE to
%   extract it.
%
%   Example:
%
%       opts.outputDir = tempdir;
%       file = publish('intro',opts);
%       web(file)
%
%   See also NOTEBOOK, GRABCODE.

% $Revision: 1.1.6.43 $  $Date: 2010/05/13 17:37:46 $
% Copyright 1984-2010 The MathWorks, Inc.

% This function requires Java.
if ~usejava('jvm')
    error('MATLAB:publish:NoJvm','PUBLISH requires Java.');
end

% Default to HTML publishing.
if (nargin < 2)
    options = 'html';
end

% If options is a simple string (format), convert to structure.
if ischar(options)
    t = options;
    options = struct;
    options.format = t;
end

% Process options.
checkOptionFields(options);
options = supplyDefaultOptions(options);
validateOptions(options)
format = options.format;

% Locate source.
fullPathToScript = locateFile(file);
if isempty(fullPathToScript)
    error('MATLAB:publish:SourceNotFound','Cannot find "%s".',file);
end
code = file2char(fullPathToScript);
[scriptDir,prefix] = fileparts(fullPathToScript);

% Determine command to run.
options = setCodeToEvaluateIfEmpty(file,options,fullPathToScript);

% Determine publish location.
if isfield(options,'outputDir') && ~isempty(options.outputDir)
    outputDir = options.outputDir;
    % Tolerate a trailing filesep, like from TEMPDIR.
    outputDir = regexprep(outputDir,'[/\\]$','');
    % Check for relative path.
    javaFile = java.io.File(outputDir);
    if (javaFile.isAbsolute)
        % Run it through FULLFILE to correct file separators, if required.
        outputDir = fullfile(outputDir);
    else
        % Turn the relative path into an absolute path.
        outputDir = fullfile(pwd,outputDir);
    end
else
    outputDir = fullfile(scriptDir,'html');
end
switch format
    case 'latex'
        ext = 'tex';
    otherwise
        ext = format;
end
outputAbsoluteFilename = fullfile(outputDir,[prefix '.' ext]);

% Make sure we can write to this filename.  Create the directory, if needed.
message = prepareOutputLocation(outputAbsoluteFilename);
if ~isempty(message)
    error('MATLAB:publish:CannotWriteOutput',strrep(message,'\','\\'))
end

% Determine where to save image files.
switch format
    case {'doc','ppt','pdf'}
        imageDir = tempdir;
        % Trim the trailing slashes off the directory to make later logic easier.
        imageDir(end) = [];
        needToCleanTempdir = true;
    otherwise
        imageDir = outputDir;
        needToCleanTempdir = false;
end

% Flush out any existing images.  This also verifies there are no read-only
% images in the way.  It also keeps us from drooling images if a
% republished version has fewer images than the existing one.
deleteExistingImages(imageDir,prefix,false)

% Convert the M-code to XML.
[dom,cellBoundaries] = m2mxdom(code);

% Add reference to original file.
newNode = dom.createElement('m-file');
newTextNode = dom.createTextNode(prefix);
newNode.appendChild(newTextNode);
dom.getFirstChild.appendChild(newNode);
newNode = dom.createElement('filename');
newTextNode = dom.createTextNode(fullPathToScript);
newNode.appendChild(newTextNode);
dom.getFirstChild.appendChild(newNode);
newNode = dom.createElement('outputdir');
newTextNode = dom.createTextNode(outputDir);
newNode.appendChild(newTextNode);
dom.getFirstChild.appendChild(newNode);

% Creat images of TeX equations for non-TeX output.
dom = createEquationImages(dom,imageDir,prefix,format,outputDir);

% Evaluate each cell, snap the output, and store the results.
if options.evalCode
    dom = evalmxdom(file,dom,cellBoundaries,prefix,imageDir,outputDir,options);
end

% Post-process the DOM.
dom = removeDisplayCode(dom,options.showCode);
dom = truncateOutput(dom,options.maxOutputLines);

% Write to the output format.
switch format
    case 'xml'
        if isempty(options.stylesheet)
            xmlwrite(outputAbsoluteFilename,dom)
        else
            xslt(dom,options.stylesheet,outputAbsoluteFilename);
        end

    case 'html'
        xslt(dom,options.stylesheet,outputAbsoluteFilename);
        
    case 'latex'
        xslt(dom,options.stylesheet,outputAbsoluteFilename);
        resaveWithNativeEncoding(outputAbsoluteFilename)
        
    case 'doc'
        mxdom2word(dom,outputAbsoluteFilename);

    case 'ppt'
        mxdom2ppt(dom,outputAbsoluteFilename);

    case 'docbook'
        xslt(dom,options.stylesheet,outputAbsoluteFilename);
        resaveWithNativeEncoding(outputAbsoluteFilename)

    case 'pdf'
        publishToPdf(dom,options,outputAbsoluteFilename)
end


% Cleanup.
if needToCleanTempdir
    try
        deleteExistingImages(imageDir,prefix,true)
    catch %#ok<CTCH>
        % Don't error if cleanup fails for some strange reason.
    end
end
if strcmp(format,'doc') && (numel(dir(fullfile(tempdir,'VBE'))) == 2)
    % Word drools this empty temporary directory.
    try
        rmdir(fullfile(tempdir,'VBE'))
    catch %#ok<CTCH>
        % Don't error if cleanup fails for some strange reason.
    end
end

%===============================================================================
function checkOptionFields(options)
validOptions = {'format','stylesheet','outputDir','imageFormat', ...
    'figureSnapMethod','useNewFigure','maxHeight','maxWidth','showCode', ...
    'evalCode','stopOnError','catchError','createThumbnail','maxOutputLines', ...
    'codeToEvaluate'};
bogusFields = setdiff(fieldnames(options),validOptions);
if ~isempty(bogusFields)
    error('MATLAB:publish:InvalidOption','Invalid option "%s".  Note that options are case sensitive.',bogusFields{1});
end

%===============================================================================
function options = supplyDefaultOptions(options)
% Supply default options for any that are missing.
if ~isfield(options,'format')
    options.format = 'html';
end
format = options.format;
if ~isfield(options,'stylesheet') || isempty(options.stylesheet)
    switch format
        case 'html'
            codepadDir = fileparts(which(mfilename));
            styleSheet = fullfile(codepadDir,'private','mxdom2simplehtml.xsl');
            options.stylesheet = styleSheet;
        case 'latex'
            codepadDir = fileparts(which(mfilename));
            styleSheet = fullfile(codepadDir,'private','mxdom2latex.xsl');
            options.stylesheet = styleSheet;
        case {'docbook','pdf'}
            codepadDir = fileparts(which(mfilename));
            styleSheet = fullfile(codepadDir,'private','mxdom2docbook.xsl');
            options.stylesheet = styleSheet;
        otherwise
            options.stylesheet = '';
    end
end
if ~isfield(options,'figureSnapMethod')
    options.figureSnapMethod = 'entireGUIWindow';
end
if ~isfield(options,'imageFormat') || isempty(options.imageFormat)
    options.imageFormat = '';
elseif strcmp(options.imageFormat,'jpg')
    options.imageFormat = 'jpeg';
elseif strcmp(options.imageFormat,'tif')
    options.imageFormat = 'tiff';
elseif strcmp(options.imageFormat,'gif')
    error('MATLAB:publish:NoGIFs','"gif" is not a supported imageFormat.');
end
if ~isfield(options,'useNewFigure')
    options.useNewFigure = true;
end
if ~isfield(options,'maxHeight')
    options.maxHeight = [];
end
if ~isfield(options,'maxWidth')
    options.maxWidth = [];
end
if ~isfield(options,'showCode')
    options.showCode = true;
end
if ~isfield(options,'evalCode')
    options.evalCode = true;
end
if ~isfield(options,'stopOnError')
    options.stopOnError = true;
end
if ~isfield(options,'catchError')
    options.catchError = true;
end
if ~isfield(options,'createThumbnail')
    options.createThumbnail = true;
end
if ~isfield(options,'maxOutputLines')
    options.maxOutputLines = Inf;
end
if ~isfield(options,'codeToEvaluate')
    options.codeToEvaluate = '';
end

%===============================================================================
function validateOptions(options)

% Check format.
supportedFormats = {'html','doc','ppt','xml','rpt','latex','pdf','docbook'};
if isempty(strmatch(options.format,supportedFormats,'exact'))
    error('MATLAB:publish:UnknownFormat','Unsupported format "%s".',options.format);
end

% Check stylesheet.
if ~isempty(options.stylesheet) && ~exist(options.stylesheet,'file')
    error( ...
        'MATLAB:publish:StylesheetNotFound', ...
        'The specified stylesheet, "%s", does not exist.', ...
        options.stylesheet)
end

% Check logical scalars.
logicalScalarOptions = {'useNewFigure','showCode','evalCode','catchError','createThumbnail'};
isLogicalScalarOrEmpty = @(x) ...
    isempty(options.(x)) || ...
    (islogical(options.(x)) && (numel(options.(x))==1));
badOptions = logicalScalarOptions(~cellfun(isLogicalScalarOrEmpty,logicalScalarOptions));
if ~isempty(badOptions)
    error( ...
        'MATLAB:publish:InvalidOptionValue', ...
        'The value of "%s" must be a logical scalar, e.g. true and not the string ''true''.', ...
        badOptions{1})
end

% Check maxOutputLines.
if ~isnumeric(options.maxOutputLines) || ...
        (numel(options.maxOutputLines) ~= 1) || ...
        (options.maxOutputLines < 0) || ...
        isnan(options.maxOutputLines) || ...
        (round(options.maxOutputLines) ~= options.maxOutputLines)
    error('MATLAB:publish:InvalidOptionValue', ...
        'The value of "maxOutputLines" must be Inf or a non-negative integer.');
end

% Check consistency.
vectorFormats = internal.matlab.publish.getVectorFormats();
if ~isempty(strmatch(options.imageFormat,vectorFormats,'exact'))
    if strcmp(options.figureSnapMethod,'getframe')
        error( ...
            'MATLAB:publish:StylesheetNotFound', ...
            'The imageFormat "%s" is incompatible with the figureSnapMethod "getframe".', ...
            options.imageFormat)
    end
    if ~isempty(options.maxHeight)
        warning('MATLAB:publish:IncompatibleOptions', ...
            'Setting a maximum image height is incompatible with %s-files and will be ignored.', ...
            upper(options.imageFormat))
    end
    if ~isempty(options.maxWidth)
        warning('MATLAB:publish:IncompatibleOptions', ...
            'Setting a maximum image width is incompatible with %s-files and will be ignored.', ...
            upper(options.imageFormat))
    end
end

% Format-specific limitations.
if strcmp(options.format,'pdf') && ...
        ~isempty(options.imageFormat) && ...
        ~(strcmp(options.imageFormat,'bmp') || strcmp(options.imageFormat,'jpeg'))
    error('MATLAB:publish:InvalidOptionValue', ...
        'PDF output only supports an imageFormat of BMP or JPEG.');
end

% Check deprication.
if ~isempty(options.stopOnError) && (options.stopOnError == false)
        warning('MATLAB:publish:DeprecatedOptions', ...
            'stopOnError is no longer supported.  Use TRY/CATCH in your code for a similar effect.')    
end


%===============================================================================
function options = setCodeToEvaluateIfEmpty(file,options,fullPathToScript)
if isempty(options.codeToEvaluate)
    cmd = regexprep(file,'.*[\\/]','');
    cmd = regexprep(cmd,'\.m$','');
    % Do a case insensitve match because the PC is case-insensitve.
    if ~strcmpi(strrep(fullPathToScript,'/',filesep),which(cmd)) && ...
        (options.evalCode==true)
        pathMessage = 'PUBLISH needs to run the file because the evalCode option is set,\nbut the file is not on the MATLAB path. \n FIRM Toolbox: If you see this error, this most probably means that you have a file with the same name on the Matlab path. You have to remove it.';
        error('MATLAB:publish:OffPath',pathMessage)
    end
    options.codeToEvaluate = cmd;
end

%===============================================================================
function deleteExistingImages(imageDir,prefix,equations)

% Start with a list of candidates for deletions.
d = dir(fullfile(imageDir,[prefix '_*.*']));

% Define the regexp to use to to lessen the chance of false hits.
tail = '\d{2,}\.[A-Za-z]+';
if equations
    tail = ['(' tail '|eq\d+\.(?:png|bmp))'];
end
imagePattern = ['^' prefix '_' tail '$'];

% We need to detect if a DELETE failed by checking WARNING.  Save the
% original state and clear the warning.
[lastmsg,lastid] = lastwarn('');

% Delete the images.
for i = 1:length(d)
    if (regexp(d(i).name,imagePattern) == 1)
        toDelete = fullfile(imageDir,d(i).name);
        delete(toDelete)
        if ~isempty(lastwarn)
            error('MATLAB:publish:CannotWriteOutput', ...
                'Cannot delete "%s".',toDelete)
        end
    end
end

% Delete the thumbnail.
thumbnail = fullfile(imageDir,[prefix '.png']);
if ~isempty(dir(thumbnail))
    delete(thumbnail)
    if ~isempty(lastwarn)
        error('MATLAB:publish:CannotWriteOutput', ...
            'Cannot delete "%s".',thumbnail)
    end
end

% Restore the warning.
lastwarn(lastmsg,lastid);

%===============================================================================
function dom = removeDisplayCode(dom,showCode)
if ~showCode
    while true
        codeNodeList = dom.getElementsByTagName('mcode');
        if (codeNodeList.getLength == 0)
            break;
        end
        codeNode = codeNodeList.item(0);
        codeNode.getParentNode.removeChild(codeNode);
    end
    while true
        codeNodeList = dom.getElementsByTagName('mcode-xmlized');
        if (codeNodeList.getLength == 0)
            break;
        end
        codeNode = codeNodeList.item(0);
        codeNode.getParentNode.removeChild(codeNode);
    end
end

%===============================================================================
function dom = truncateOutput(dom,maxOutputLines)
if ~isinf(maxOutputLines)
    outputNodeList = dom.getElementsByTagName('mcodeoutput');
    % Start at the end in case we remove nodes.
    for iOutputNodeList = outputNodeList.getLength:-1:1
        outputNode = outputNodeList.item(iOutputNodeList-1);
        if (maxOutputLines == 0)
            outputNode.getParentNode.removeChild(outputNode);
        else
            text = char(outputNode.getFirstChild.getData);
            newlines = regexp(text,'\n');
            if maxOutputLines <= length(newlines)
                chopped = text(newlines(maxOutputLines):end);
                text = text(1:newlines(maxOutputLines));
                if ~isempty(regexp(chopped,'\S','once'))
                    text = [text '...']; %#ok<AGROW>
                end
            end
            outputNode.getFirstChild.setData(text);
        end
    end
end

%===============================================================================
function resaveWithNativeEncoding(outputAbsoluteFilename)
% UTF in.
f = fopen(outputAbsoluteFilename,'r','n','UTF-8');
c = fread(f,'char=>char')';
fclose(f);

% Native out.
f = fopen(outputAbsoluteFilename,'w');
fwrite(f,c,'char');
fclose(f);

%===============================================================================
function publishToPdf(dom,options,outputAbsoluteFilename)

% Unix doesn't figure out when these are full paths.  Help it out.
if ~ispc
    imgNodeList = dom.getElementsByTagName('img');
    for i = 1:imgNodeList.getLength();
        node = imgNodeList.item(i-1);
        src = char(node.getAttribute('src'));
        if strmatch('/',src)
            node.setAttribute('src',file2urn(src));
        end
    end
end

% Create the temporary DocBook.
docbook = xslt(dom,options.stylesheet,'-tostring');

% Driver, set log level and set to render PDF.
fopDriver = org.apache.fop.apps.Driver;
logger = org.apache.avalon.framework.logger.ConsoleLogger( ...
    org.apache.avalon.framework.logger.ConsoleLogger.LEVEL_WARN);
fopDriver.setLogger(logger);
org.apache.fop.messaging.MessageHandler.setScreenLogger(logger);
fopDriver.setRenderer(fopDriver.RENDER_PDF);

% Output.
fopOutputStream = java.io.FileOutputStream(outputAbsoluteFilename);
fopDriver.setOutputStream(fopOutputStream);
targetName = javax.xml.transform.sax.SAXResult(fopDriver.getContentHandler);

% Input.
saxParserFactory = javax.xml.parsers.SAXParserFactory.newInstance;
saxParserFactory.setValidating(false);
saxParserFactory.setNamespaceAware(true);
xmlReader = saxParserFactory.newSAXParser.getXMLReader();
uriResolver = com.mathworks.toolbox.rptgencore.tools.UriResolverRG();
xmlReader.setEntityResolver(uriResolver);
saxInputSource = org.xml.sax.InputSource(java.io.StringReader(docbook));
saxSource = javax.xml.transform.sax.SAXSource(xmlReader,saxInputSource);

% Transform.
noToc = dom.getElementsByTagName('steptitle').getLength < 3;
xslt(saxSource,getPdfStylesheet(noToc),targetName);

% Cleanup.
fopOutputStream.close;
org.apache.fop.image.FopImageFactory.resetCache();

%===============================================================================
function styleDom = getPdfStylesheet(noToc)
% Stylesheet
styleDom = com.mathworks.xml.XMLUtils.createDocument('xsl:stylesheet');
de = styleDom.getDocumentElement();
de.setAttribute('xmlns:xsl','http://www.w3.org/1999/XSL/Transform');
de.setAttribute('xmlns','http://www.w3.org/TR/xhtml1/transitional');
de.setAttribute('version','1.0');
importNode = styleDom.createElement('xsl:import');
xslUrl = file2urn(fullfile(matlabroot, ...
    '/sys/namespace/docbook/v4/xsl/fo/docbook_rptgen.xsl'));
importNode.setAttribute('href',xslUrl);
de.appendChild(importNode);
addVariable(styleDom,de,'show.comments','0')
addVariable(styleDom,de,'fop.extensions','1')
if noToc
    addVariable(styleDom,de,'generate.toc','0')
end

% Fonts
if strncmp(get(0,'Language'),'ja',2)
    fontName = 'MS Gothic';
    addVariableText(styleDom,de,'body.font.family',fontName)
    addVariableText(styleDom,de,'monospace.font.family',fontName)
    addVariableText(styleDom,de,'title.font.family',fontName)
    locRegisterFopFonts(fontName)
end

% Format hyperlinks.
addVariable(styleDom,de,'ulink.show','0')
attributeSet = styleDom.createElement('xsl:attribute-set');
attributeSet.setAttribute('name','xref.properties');
de.appendChild(attributeSet);
addAttribute(styleDom,attributeSet,'text-decoration','underline')
addAttribute(styleDom,attributeSet,'color','blue')

%===============================================================================
function addVariable(dom,node,name,value)
var = dom.createElement('xsl:variable');
var.setAttribute('name',name);
var.setAttribute('select',value);
node.appendChild(var);

%===============================================================================
function addVariableText(dom,node,name,value)
var = dom.createElement('xsl:variable');
var.setAttribute('name',name);
node.appendChild(var);
var.appendChild(dom.createTextNode(value));

%===============================================================================
function addAttribute(dom,attributeSet,name,value)
attribute = dom.createElement('xsl:attribute');
attribute.setAttribute('name',name);
attribute.appendChild(dom.createTextNode(value));
attributeSet.appendChild(attribute);

%===============================================================================
function urnFile = file2urn(fileName)
%FILE2URN converts a file name to a Universal Resource Name
%   URN = FILE2URN(FILENAME) where FILENAME is a full path to a file

if strncmp(fileName,'file:///',8)
    % Test: c:/foo/bar -> file:///c:/foo/bar
    urnFile = fileName;

else
    % RFC 2141 URN Syntax specifies "%" "/" "?" "#" as key characters.
    % We do not need to escape the character "/" because file systems do not allow
    % this characters in directory names.
    fileName = strrep(fileName,'%','%25');
    fileName = strrep(fileName,'?','%3F');
    fileName = strrep(fileName,'#','%23');
    fileName = strrep(fileName,' ','%20');

    if strncmp(fileName,'/',1)
        % Test: /root/dir/file -> file:///root/dir/file
        % Test: /root/folder with space/file -> 
        %       file:///root/dir/folder%20with%20space/file
        fileName = strrep(fileName,'\','/');
        urnFile = ['file://' fileName];
    else
        % Test: \\server\root\dir\dir -> file://///server/root/dir/dir
        % Test: c:\dir\dir            -> file:///c:/dir/dir
        fileName = strrep(fileName,'\','/');
        urnFile = ['file:///' fileName];
    end
end

%===============================================================================
function locRegisterFopFonts(fontName)

[fontFile, fontMetricsFile] = locGetFontInfo(fontName);
[~, fontMetricsName] = fileparts(fontMetricsFile);

% Static, returns list
fontList = org.apache.fop.configuration.Configuration.getFonts();

if isempty(fontList)
    fontList = java.util.ArrayList(10);
    % configuration.STANDARD
    org.apache.fop.configuration.Configuration.put('fonts',fontList);
    addFont = true;
else
    % Check to see if the font is already there
    addFont = true;
    li = fontList.listIterator;
    while addFont && li.hasNext
        thisFont = li.next;
        if ~isempty(thisFont) && isa(thisFont,'org.apache.fop.configuration.FontInfo')
            try %#ok
                addFont = ~thisFont.getMetricsFile().getPath().endsWith(fontMetricsName);
            end
        end
    end
end

if addFont
    fontTriplets = java.util.ArrayList(5);
    fontTriplets.add(org.apache.fop.configuration.FontTriplet(...
        fontName,... % name
        'normal',... % weight
        'normal'));  % style
    fontTriplets.add(org.apache.fop.configuration.FontTriplet(...
        fontName,... % name
        'normal',... % weight
        'italic'));  % style
    fontTriplets.add(org.apache.fop.configuration.FontTriplet(...
        fontName,... % name
        'bold',...   % weight
        'normal'));  % style
    fontTriplets.add(org.apache.fop.configuration.FontTriplet(...
        fontName,... % name
        'bold',...   % weight
        'italic'));  % style
    
    fontList.add(org.apache.fop.configuration.FontInfo(...
        fontName,...        % String name
        fontMetricsFile,... % String metricsFile
        true,...            % Boolean kerning
        fontTriplets,...    % List fontTriplets
        fontFile));         % String embedFile
end

%===============================================================================
function [fontFile fontMetricsFile] = locGetFontInfo(fontName)
% We only support MS Gothic.

fontFile = '';
fontMetricsFile = '';

if strcmpi(fontName,'MS Gothic')
    % Get font file
    winDir = getenv('windir');
    if isempty(winDir)
        winDir = 'c:\windows';
    end
    fontFile = fullfile(winDir,'fonts','msgothic.ttc');
    if ~exist(fontFile,'file')
        % Windows may know about the font file location anyway.  Proceed and
        % hope for the best.
        fontFile = '';
    end
    
    % Get font metrics file
    if locIsWinVista()
        metricsName = 'MS_Gothic_vista.xml';
    else
        metricsName = 'MS_Gothic.xml';
    end
    fontMetricsFile = fullfile(matlabroot,'sys','namespace','docbook','v4','xsl', ...
        'tools','fontmetrics',metricsName);
end

%===============================================================================
function tf = locIsWinVista
if ~isunix
    windowsName = winqueryreg('HKEY_LOCAL_MACHINE',...
        'SOFTWARE\Microsoft\Windows NT\CurrentVersion',...
        'ProductName');
    windowsName = lower(windowsName);
    tf = ~isempty(strfind(windowsName,'vista'));
else
    tf = false;
end

%===============================================================================
% All these subfunctions are for equation handling.
%===============================================================================
function dom = createEquationImages(dom,imageDir,prefix,format,outputDir)
% Render equations as images to be included in the document.

switch format
    case 'latex'
        return
    case {'docbook','pdf'}
        ext = '.bmp';
    otherwise
        ext = '.png';
end

% Setup.
baseImageName = fullfile(imageDir,prefix);
[tempfigure,temptext] = getRenderingFigure;

% Loop over each equation.
equationList = dom.getElementsByTagName('equation');
for i = 1:getLength(equationList)
    equationNode = equationList.item(i-1);
    equationText = char(equationNode.getTextContent);
    fullFilename = [baseImageName '_' hashEquation(equationText) ext];
    % Check to see if this equation needs to be rendered.
    if ~isempty(dir(fullFilename))
        % We've already got it on disk.  Use it.
        [height,width,~] = size(imread(fullFilename));
        swapTexForImg(dom,equationNode,outputDir,fullFilename,equationText,width,height)
    else
        % We need to render it.
        [x,texWarning] = renderTex(equationText,tempfigure,temptext);
        if isempty(texWarning)
            % Now shrink it down to get anti-aliasing.
            newSize = ceil(size(x)/2);
            x = internal.matlab.publish.make_thumbnail(x,newSize(1:2));
            % Rendering succeeded.  Write out the image and use it.
            imwrite(x,fullFilename)
            % Put a link to the image in the DOM.
            swapTexForImg(dom,equationNode,outputDir,fullFilename,equationText,newSize(2),newSize(1))
        else
            % Rendering failed.  Add error message.
            beep
            errorNode = dom.createElement('pre');
            errorNode.setAttribute('class','error')
            errorNode.appendChild(dom.createTextNode(texWarning));
            % Insert the error after the equation.  This would be easier if
            % there were an insertAfter node method.
            pNode = equationNode.getParentNode;
            if isempty(pNode.getNextSibling)
                pNode.getParentNode.appendChild(errorNode);
            else
                pNode.getParentNode.insertBefore(errorNode,pNode.getNextSibling);
            end
        end
    end
end

% Cleanup.
close(tempfigure)

%===============================================================================
function swapTexForImg(dom,equationNode,outputDir,fullFilename,equationText,width,height)
% Swap the TeX equation for the IMG.
equationNode.removeChild(equationNode.getFirstChild);
imgNode = dom.createElement('img');
imgNode.setAttribute('alt',equationText);
imgNode.setAttribute('src',strrep(fullFilename,[outputDir filesep],''));
imgNode.setAttribute('class','equation');
imgNode.setAttribute('width',num2str(width));
imgNode.setAttribute('height',num2str(height));
equationNode.appendChild(imgNode);


%===============================================================================
function [tempfigure,temptext] = getRenderingFigure

% Create a figure for rendering the equation, if needed.
tag = ['helper figure for ' mfilename];
tempfigure = findall(0,'type','figure','tag',tag);
if isempty(tempfigure)
    figurePos = get(0,'ScreenSize');
    if ispc
        % Set it off-screen since we have to make it visible before printing.
        % Move it over and down plus a little bit to keep the edge from showing.
        figurePos(1:2) = figurePos(3:4)+100;
    end
    % Create a new figure.
    tempfigure = figure( ...
        'HandleVisibility','off', ...
        'IntegerHandle','off', ...
        'Visible','off', ...
        'PaperPositionMode', 'auto', ...
        'PaperOrientation', 'portrait', ...
        'Color','w', ...
        'Position',figurePos, ...
        'Tag',tag);
    tempaxes = axes('position',[0 0 1 1], ...
        'Parent',tempfigure, ...
        'XTick',[],'ytick',[], ...
        'XLim',[0 1],'ylim',[0 1], ...
        'Visible','off');
    temptext = text('Parent',tempaxes,'Position',[.5 .5], ...
        'HorizontalAlignment','center','FontSize',22, ...
        'Interpreter','latex');
else
    % Use existing figure.
    tempaxes = findobj(tempfigure,'type','axes');
    temptext = findobj(tempaxes,'type','text');
end

%===============================================================================
function [x,texWarning] = renderTex(equationText,tempfigure,temptext)

% Setup.
[lastMsg,lastId] = lastwarn('');
set(temptext,'string',strrep(equationText,char(10),' '));
if ispc
    % The font metrics are not set properly unless the figure is visible.
    set(tempfigure,'Visible','on');
end
drawnow;

% Snap.
x = hardcopy(tempfigure,'-dzbuffer','-r0');
set(tempfigure,'Visible','off');
texWarning = lastwarn;
lastwarn(lastMsg,lastId)
set(temptext,'string','');

% Trim, but keep the baseline-adjusted-middle in the middle.
if isempty(texWarning)
    % Sometimes the first pixel isn't white.  Crop that out.
    x(1,:,:) = [];
    x(:,1,:) = [];
    % Crop out the rest of the whitespace border.
    [i,j] = find(sum(double(x),3)~=765);
    x = x(min(i):max(i),min(j):max(j),:);
    if isempty(x)
        % The image is empty.  Return something so IMWRITE doesn't complain.
        x = 255*ones(1,3,'uint8');
    end
end

%===============================================================================
%===============================================================================
