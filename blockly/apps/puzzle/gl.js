// This file was automatically generated from template.soy.
// Please don't edit this file by hand.

if (typeof puzzlepage == 'undefined') { var puzzlepage = {}; }


puzzlepage.messages = function(opt_data, opt_ignored, opt_ijData) {
  return '<div id="MSG" style="display: none"><span id="country1">Australia</span><span id="country1Flag">flag_au.png</span><span id="country1FlagHeight">50</span><span id="country1FlagWidth">100</span><span id="country1Language">inglés</span><span id="country1City1">Melbourne</span><span id="country1City2">Sidney</span><span id="country1HelpUrl">http://gl.wikipedia.org/wiki/Australia</span><span id="country2">Alemaña</span><span id="country2Flag">flag_de.png</span><span id="country2FlagHeight">60</span><span id="country2FlagWidth">100</span><span id="country2Language">alemán</span><span id="country2City1">Berlín</span><span id="country2City2">Múnic</span><span id="country2HelpUrl">http://gl.wikipedia.org/wiki/Alema%C3%B1a__Deutschland</span><span id="country3">China</span><span id="country3Flag">flag_cn.png</span><span id="country3FlagHeight">66</span><span id="country3FlagWidth">100</span><span id="country3Language">chinés</span><span id="country3City1">Pequín</span><span id="country3City2">Shanghai</span><span id="country3HelpUrl">http://gl.wikipedia.org/wiki/Rep%C3%BAblica_Popular_da_China</span><span id="country4">Brasil</span><span id="country4Flag">flag_br.png</span><span id="country4FlagHeight">70</span><span id="country4FlagWidth">100</span><span id="country4Language">portugués</span><span id="country4City1">Río de Xaneiro</span><span id="country4City2">San Paulo</span><span id="country4HelpUrl">http://gl.wikipedia.org/wiki/Brasil</span><span id="flag">bandeira:</span><span id="language">lingua:</span><span id="languageChoose">escolle...</span><span id="cities">cidades:</span><span id="error0">Perfecto!\nOs %1 bloques son correctos.</span><span id="error1">Por pouco! Un bloque é incorrecto.</span><span id="error2">%1 bloques son incorrectos.</span><span id="tryAgain">O bloque destacado non é correcto.\nSigue intentándoo.</span></div>';
};


puzzlepage.start = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<table id="header" width="100%"><tr><td valign="bottom"><h1><span id="title"><a href="../index.html">Blockly</a> : Crebacabezas</span></h1></td><td><select id="languageMenu" onchange="BlocklyApps.changeLanguage();"></select>&nbsp; &nbsp;<button id="helpButton" onclick="Puzzle.showHelp();">Axuda</button>&nbsp; &nbsp;<button id="checkButton" class="launch" onclick="Puzzle.checkAnswers();">Comprobar as respostas</button></td></tr></table><script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="shadow"></div><div id="help"><div style="padding-bottom: 0.7ex">A cada país (en verde) faille corresponder a súa bandeira, sinala a súa lingua e apiña as súas cidades.</div><iframe src="help.html?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '" style="height: 200px; width: 100%; border: none;"></iframe><div class="farSide" style="padding: 1ex 3ex 0"><button id="okButton" onclick="Puzzle.hideHelp()">Aceptar</button></div></div>';
};


puzzlepage.help = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="notouch"></div>';
};
