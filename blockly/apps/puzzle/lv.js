// This file was automatically generated from template.soy.
// Please don't edit this file by hand.

if (typeof puzzlepage == 'undefined') { var puzzlepage = {}; }


puzzlepage.messages = function(opt_data, opt_ignored, opt_ijData) {
  return '<div id="MSG" style="display: none"><span id="country1">Austrālija</span><span id="country1Flag">flag_au.png</span><span id="country1FlagHeight">50</span><span id="country1FlagWidth">100</span><span id="country1Language">Angļu</span><span id="country1City1">Melburna</span><span id="country1City2">Sidneja</span><span id="country1HelpUrl">http://en.wikipedia.org/wiki/Australia</span><span id="country2">Vācija</span><span id="country2Flag">flag_de.png</span><span id="country2FlagHeight">60</span><span id="country2FlagWidth">100</span><span id="country2Language">Vācu</span><span id="country2City1">Berlīne</span><span id="country2City2">Minhene</span><span id="country2HelpUrl">http://en.wikipedia.org/wiki/Germany</span><span id="country3">Ķīna</span><span id="country3Flag">flag_cn.png</span><span id="country3FlagHeight">66</span><span id="country3FlagWidth">100</span><span id="country3Language">Ķīniešu</span><span id="country3City1">Pekina</span><span id="country3City2">Šanhaja</span><span id="country3HelpUrl">http://en.wikipedia.org/wiki/China</span><span id="country4">Brazīlija</span><span id="country4Flag">flag_br.png</span><span id="country4FlagHeight">70</span><span id="country4FlagWidth">100</span><span id="country4Language">Portugāļu</span><span id="country4City1">Riodežaneiro</span><span id="country4City2">Sanpaulu</span><span id="country4HelpUrl">http://en.wikipedia.org/wiki/Brazil</span><span id="flag">karogs:</span><span id="language">valoda:</span><span id="languageChoose">izvēlies...</span><span id="cities">pilsētas:</span><span id="error0">Lieliski!\nVisi %1 bloki ir pareizi.</span><span id="error1">Gandrīz! Viens bloks nav pareizs.</span><span id="error2">%1 bloki nav pareizi.</span><span id="tryAgain">Iezīmētais bloks nav pareizs.\nMēģini vēl.</span></div>';
};


puzzlepage.start = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<table id="header" width="100%"><tr><td valign="bottom"><h1><span id="title"><a href="../index.html">Blockly</a> : Saliekamattēls</span></h1></td><td><select id="languageMenu" onchange="BlocklyApps.changeLanguage();"></select>&nbsp; &nbsp;<button id="helpButton" onclick="Puzzle.showHelp();">Palīdzība</button>&nbsp; &nbsp;<button id="checkButton" class="launch" onclick="Puzzle.checkAnswers();">Pārbaudīt atbildes</button></td></tr></table><script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="shadow"></div><div id="help"><div style="padding-bottom: 0.7ex">Katrai valstij (zaļas), pievieno tās karogu, izvēlies tās valodu un saliec tās pilsētas stabiņā.</div><iframe src="help.html?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '" style="height: 200px; width: 100%; border: none;"></iframe><div class="farSide" style="padding: 1ex 3ex 0"><button id="okButton" onclick="Puzzle.hideHelp()">Labi</button></div></div>';
};


puzzlepage.help = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="notouch"></div>';
};
