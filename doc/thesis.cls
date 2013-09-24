\ProvidesClass{thesis}

% based on the report class
\LoadClass[11pt, twoside, a4paper]{report}

% move extra whitespace to the bottom
% instead of spreading it between paragraphs
\raggedbottom

% XeTeX
\RequirePackage{ifxetex}
\RequireXeTeX

% BibTeX
\RequirePackage{cite}

% make Bibliography appear in ToC
\RequirePackage[nottoc,numbib]{tocbibind}

% colored tables
\RequirePackage[table]{xcolor}
\rowcolors{0}{white}{black!6}

% 2Dos
\RequirePackage{todonotes}

% captions
\RequirePackage{caption}
\captionsetup[figure]{labelfont=bf}
\captionsetup[lstlisting]{labelfont=bf}
\captionsetup[table]{labelfont=bf}

% float environments
\RequirePackage{float}
\RequirePackage{placeins}

% set list of figures to appear as section in toc
\renewcommand\listoffigures{%
  \section{\listfigurename}%
  \@mkboth{\MakeUppercase\listfigurename}%
          {\MakeUppercase\listfigurename}%
  \@starttoc{lof}%
}

% math environments
\RequirePackage{amsmath}

% tables
\RequirePackage{tabularx}
\newcolumntype{L}[1]{>{\hsize=#1\hsize\raggedright\arraybackslash}X}
\newcolumntype{R}[1]{>{\hsize=#1\hsize\raggedleft\arraybackslash}X}
\newcolumntype{C}[2]{>{\hsize=#1\hsize\columncolor{#2}\centering\arraybackslash}X}

\newcommand{\thickline} {%
  \noalign {\ifnum 0=`}\fi \hrule height 2pt
  \futurelet \reserved@a \@xhline
}

% source code listings
\RequirePackage{listings}

% change list of listings name to match list of figures name
\renewcommand{\lstlistlistingname}{List of Listings}

% set list of listings to appear as section in toc
\renewcommand\lstlistoflistings{%
  \section{\lstlistlistingname}%
  \@mkboth{\MakeUppercase\lstlistlistingname}%
          {\MakeUppercase\lstlistlistingname}%
  \@starttoc{lol}%
}

% set list of tables to appear as section in toc
\renewcommand\listoftables{%
  \section{\listtablename}%
  \@mkboth{\MakeUppercase\listtablename}%
          {\MakeUppercase\listtablename}%
  \@starttoc{lot}%
}

% C++ source code listing style
\lstdefinestyle{customc}{
  belowcaptionskip=1\baselineskip,
  breaklines=true,
  frame=bt,
  xleftmargin=\parindent,
  language=C,
  showstringspaces=false,
  basicstyle=\normalsize\ttfamily,
  keywordstyle=\bfseries\color{green!40!black},
  commentstyle=\itshape\color{purple!40!black},
  identifierstyle=\color{blue},
  stringstyle=\color{orange},
}
\lstset{escapechar=@, style=customc}

% TikZ
\RequirePackage{verbatim}
\RequirePackage{tikz}
\usetikzlibrary{arrows, shapes, chains}

% custom fonts
\RequirePackage{fontspec}

% sans serif font
\setsansfont[
  Scale=1.00,
  Mapping=tex-text,
  ExternalLocation=./fonts/,
  BoldFont={HelveticaLTStd_Bold.otf},
  ItalicFont={HelveticaLTStd_Italic.otf},
  BoldItalicFont={HelveticaLTStd_BoldItalic.otf}
]{HelveticaLTStd.otf}

% monospaced font
\setmonofont[
  Scale=1.0,
  Mapping=tex-text,
  ExternalLocation=./fonts/,
  BoldFont={CourierPrime_Bold.ttf},
  ItalicFont={CourierPrime_Italic.ttf},
  BoldItalicFont={CourierPrime_BoldItalic.ttf},
  HyphenChar={-}
]{CourierPrime.ttf}

% URLs
\RequirePackage{hyperref}
