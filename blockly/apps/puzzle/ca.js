// This file was automatically generated from template.soy.
// Please don't edit this file by hand.

if (typeof puzzlepage == 'undefined') { var puzzlepage = {}; }


puzzlepage.messages = function(opt_data, opt_ignored, opt_ijData) {
  return '<div id="MSG" style="display: none"><span id="country1">Austràlia</span><span id="country1Flag">flag_au.png</span><span id="country1FlagHeight">50</span><span id="country1FlagWidth">100</span><span id="country1Language">Anglès</span><span id="country1City1">Melbourne</span><span id="country1City2">Sydney</span><span id="country1HelpUrl">http://en.wikipedia.org/wiki/Australia</span><span id="country2">Alemanya</span><span id="country2Flag">flag_de.png</span><span id="country2FlagHeight">60</span><span id="country2FlagWidth">100</span><span id="country2Language">Alemany</span><span id="country2City1">Berlín</span><span id="country2City2">Munich</span><span id="country2HelpUrl">http://en.wikipedia.org/wiki/Germany</span><span id="country3">Xina</span><span id="country3Flag">flag_cn.png</span><span id="country3FlagHeight">66</span><span id="country3FlagWidth">100</span><span id="country3Language">Xinès</span><span id="country3City1">Beijing</span><span id="country3City2">Shanghai</span><span id="country3HelpUrl">http://en.wikipedia.org/wiki/China</span><span id="country4">Brasil</span><span id="country4Flag">flag_br.png</span><span id="country4FlagHeight">70</span><span id="country4FlagWidth">100</span><span id="country4Language">Portuguès</span><span id="country4City1">Rio de Janeiro</span><span id="country4City2">S\\u00e3o Paulo</span><span id="country4HelpUrl">http://en.wikipedia.org/wiki/Brazil</span><span id="flag">Bandera:</span><span id="language">Llengua:</span><span id="languageChoose">triar...</span><span id="cities">Ciutats:</span><span id="error0">Perfecte!\nTots els blocs de %1 són correctes.</span><span id="error1">Gairebé! Una bloc és incorrecte.</span><span id="error2">%1 blocs són errònies.</span><span id="tryAgain">El bloc de subratllat no és correcta.\nContinuï provant.</span></div>';
};


puzzlepage.start = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<table id="header" width="100%"><tr><td valign="bottom"><h1><span id="title"><a href="../index.html">Blockly</a> : Trencaclosques</span></h1></td><td><select id="languageMenu" onchange="BlocklyApps.changeLanguage();"></select>&nbsp; &nbsp;<button id="helpButton" onclick="Puzzle.showHelp();">Ajuda</button>&nbsp; &nbsp;<button id="checkButton" class="launch" onclick="Puzzle.checkAnswers();">Comprovar les respostes</button></td></tr></table><script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="shadow"></div><div id="help"><div style="padding-bottom: 0.7ex">Per a cada país (verd), adjuntar la seva bandera, escollir la seva llengua i fer una pila de les seves ciutats.</div><iframe src="help.html?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '" style="height: 200px; width: 100%; border: none;"></iframe><div class="farSide" style="padding: 1ex 3ex 0"><button id="okButton" onclick="Puzzle.hideHelp()">D\'acord</button></div></div>';
};


puzzlepage.help = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="notouch"></div>';
};
