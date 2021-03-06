// This file was automatically generated from template.soy.
// Please don't edit this file by hand.

if (typeof puzzlepage == 'undefined') { var puzzlepage = {}; }


puzzlepage.messages = function(opt_data, opt_ignored, opt_ijData) {
  return '<div id="MSG" style="display: none"><span id="country1">Австралия</span><span id="country1Flag">flag_au.png</span><span id="country1FlagHeight">50</span><span id="country1FlagWidth">100</span><span id="country1Language">Англисче</span><span id="country1City1">Мельбурн</span><span id="country1City2">Сидней</span><span id="country1HelpUrl">http://ky.wikipedia.org/wiki/Австралия</span><span id="country2">Германия</span><span id="country2Flag">flag_de.png</span><span id="country2FlagHeight">60</span><span id="country2FlagWidth">100</span><span id="country2Language">Немисче</span><span id="country2City1">Берлин</span><span id="country2City2">Мюнхен</span><span id="country2HelpUrl">http://ky.wikipedia.org/wiki/Германия</span><span id="country3">Кытай</span><span id="country3Flag">flag_cn.png</span><span id="country3FlagHeight">66</span><span id="country3FlagWidth">100</span><span id="country3Language">Кытайча</span><span id="country3City1">Бээжин</span><span id="country3City2">Шанхай</span><span id="country3HelpUrl">http://ky.wikipedia.org/wiki/Кытай</span><span id="country4">Бразилия</span><span id="country4Flag">flag_br.png</span><span id="country4FlagHeight">70</span><span id="country4FlagWidth">100</span><span id="country4Language">Португалча</span><span id="country4City1">Рио-де-Жанейро</span><span id="country4City2">Сан-Паулу</span><span id="country4HelpUrl">http://ky.wikipedia.org/wiki/Бразилия</span><span id="flag">туу:</span><span id="language">Тили:</span><span id="languageChoose">тандаңыз...</span><span id="cities">шаарлары:</span><span id="error0">Мыкты!\nБардык блоктор %1 туура жайгашкан.</span><span id="error1">Аз калды!\nБир блок гана туура эмес жайгашкан.</span><span id="error2">Бир нече блок (%1) туура эмес жайгашкан.</span><span id="tryAgain">Белгиленген блок туура эмес жайгашкан.\nДагы аракет кылыңыз.</span></div>';
};


puzzlepage.start = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<table id="header" width="100%"><tr><td valign="bottom"><h1><span id="title"><a href="../index.html">Blockly</a> : Баш катырма.</span></h1></td><td><select id="languageMenu" onchange="BlocklyApps.changeLanguage();"></select>&nbsp; &nbsp;<button id="helpButton" onclick="Puzzle.showHelp();">Жардам</button>&nbsp; &nbsp;<button id="checkButton" class="launch" onclick="Puzzle.checkAnswers();">Жоопторду текшерүү</button></td></tr></table><script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="shadow"></div><div id="help"><div style="padding-bottom: 0.7ex">Ар бир өлкөгө (жашыл), туусун кадаңыз, расмий тилин тандаңыз, жана ал өлкөнүн шаарларын белгилеңиз.</div><iframe src="help.html?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '" style="height: 200px; width: 100%; border: none;"></iframe><div class="farSide" style="padding: 1ex 3ex 0"><button id="okButton" onclick="Puzzle.hideHelp()">OK</button></div></div>';
};


puzzlepage.help = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="notouch"></div>';
};
