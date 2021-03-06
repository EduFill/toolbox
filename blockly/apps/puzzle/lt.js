// This file was automatically generated from template.soy.
// Please don't edit this file by hand.

if (typeof puzzlepage == 'undefined') { var puzzlepage = {}; }


puzzlepage.messages = function(opt_data, opt_ignored, opt_ijData) {
  return '<div id="MSG" style="display: none"><span id="country1">Australija</span><span id="country1Flag">flag_au.png</span><span id="country1FlagHeight">50</span><span id="country1FlagWidth">100</span><span id="country1Language">Anglų</span><span id="country1City1">Melburnas</span><span id="country1City2">Sidnėjus</span><span id="country1HelpUrl">http://lt.wikipedia.org/wiki/Australija</span><span id="country2">Vokietija</span><span id="country2Flag">flag_de.png</span><span id="country2FlagHeight">60</span><span id="country2FlagWidth">100</span><span id="country2Language">Vokiečių</span><span id="country2City1">Berlynas</span><span id="country2City2">Miunchenas</span><span id="country2HelpUrl">http://lt.wikipedia.org/wiki/Vokietija</span><span id="country3">Kinija</span><span id="country3Flag">flag_cn.png</span><span id="country3FlagHeight">66</span><span id="country3FlagWidth">100</span><span id="country3Language">Kinų</span><span id="country3City1">Beijingas</span><span id="country3City2">Šanchajus</span><span id="country3HelpUrl">http://lt.wikipedia.org/wiki/Kinija</span><span id="country4">Brazilija</span><span id="country4Flag">flag_br.png</span><span id="country4FlagHeight">70</span><span id="country4FlagWidth">100</span><span id="country4Language">Portugalų</span><span id="country4City1">Rio de Žaneiras</span><span id="country4City2">San Paulas</span><span id="country4HelpUrl">http://lt.wikipedia.org/wiki/Brazilija</span><span id="flag">vėliava:</span><span id="language">kalba:</span><span id="languageChoose">pasirinkite...</span><span id="cities">miestai:</span><span id="error0">Puiku!\nVisi %1 blokai yra teisingi</span><span id="error1">Beveik! Vienas blokas yra neteisingas.</span><span id="error2">%1 blokai yra neteisingi.</span><span id="tryAgain">Paryškintas blokas nėra teisingas.\nToliau bandykite.</span></div>';
};


puzzlepage.start = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<table id="header" width="100%"><tr><td valign="bottom"><h1><span id="title"><a href="../index.html">Blockly</a> : Galvosūkis</span></h1></td><td><select id="languageMenu" onchange="BlocklyApps.changeLanguage();"></select>&nbsp; &nbsp;<button id="helpButton" onclick="Puzzle.showHelp();">Pagalba</button>&nbsp; &nbsp;<button id="checkButton" class="launch" onclick="Puzzle.checkAnswers();">Pasitikrinti atsakymus</button></td></tr></table><script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="shadow"></div><div id="help"><div style="padding-bottom: 0.7ex">Kiekvienai šaliai (žalia), pridėkite vėliavą, pasirinkite savo kalbą, ir padarykite savo miestų kaminą.</div><iframe src="help.html?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '" style="height: 200px; width: 100%; border: none;"></iframe><div class="farSide" style="padding: 1ex 3ex 0"><button id="okButton" onclick="Puzzle.hideHelp()">Gerai</button></div></div>';
};


puzzlepage.help = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="notouch"></div>';
};
