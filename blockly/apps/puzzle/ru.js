// This file was automatically generated from template.soy.
// Please don't edit this file by hand.

if (typeof puzzlepage == 'undefined') { var puzzlepage = {}; }


puzzlepage.messages = function(opt_data, opt_ignored, opt_ijData) {
  return '<div id="MSG" style="display: none"><span id="country1">Австралия</span><span id="country1Flag">flag_au.png</span><span id="country1FlagHeight">50</span><span id="country1FlagWidth">100</span><span id="country1Language">Английский</span><span id="country1City1">Мельбурн</span><span id="country1City2">Сидней</span><span id="country1HelpUrl">http://ru.wikipedia.org/wiki/Австралия</span><span id="country2">Германия</span><span id="country2Flag">flag_de.png</span><span id="country2FlagHeight">60</span><span id="country2FlagWidth">100</span><span id="country2Language">Немецкий</span><span id="country2City1">Берлин</span><span id="country2City2">Мюнхен</span><span id="country2HelpUrl">http://ru.wikipedia.org/wiki/Германия</span><span id="country3">Китай</span><span id="country3Flag">flag_cn.png</span><span id="country3FlagHeight">66</span><span id="country3FlagWidth">100</span><span id="country3Language">Китайский</span><span id="country3City1">Пекин</span><span id="country3City2">Шанхай</span><span id="country3HelpUrl">http://ru.wikipedia.org/wiki/Китай</span><span id="country4">Бразилия</span><span id="country4Flag">flag_br.png</span><span id="country4FlagHeight">70</span><span id="country4FlagWidth">100</span><span id="country4Language">Португальский</span><span id="country4City1">Рио-де-Жанейро</span><span id="country4City2">Сан-Паулу</span><span id="country4HelpUrl">http://ru.wikipedia.org/wiki/Бразилия</span><span id="flag">флаг:</span><span id="language">язык:</span><span id="languageChoose">выберите...</span><span id="cities">города:</span><span id="error0">Идеально!\nВсе блоки (%1) расположены правильно.</span><span id="error1">Почти! Один блок расположен неправильно.</span><span id="error2">Несколько блоков (%1) расположены неправильно.</span><span id="tryAgain">Выделенный блок расположен неправильно.\nПопробуйте ещё.</span></div>';
};


puzzlepage.start = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<table id="header" width="100%"><tr><td valign="bottom"><h1><span id="title"><a href="../index.html">Blockly</a> : Головоломка</span></h1></td><td><select id="languageMenu" onchange="BlocklyApps.changeLanguage();"></select>&nbsp; &nbsp;<button id="helpButton" onclick="Puzzle.showHelp();">Помощь</button>&nbsp; &nbsp;<button id="checkButton" class="launch" onclick="Puzzle.checkAnswers();">Проверить Ответы</button></td></tr></table><script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="shadow"></div><div id="help"><div style="padding-bottom: 0.7ex">Для каждой страны (зеленый), прикрепите её флаг, выберите официальный язык, и укажите города, находящиеся в этой стране.</div><iframe src="help.html?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '" style="height: 200px; width: 100%; border: none;"></iframe><div class="farSide" style="padding: 1ex 3ex 0"><button id="okButton" onclick="Puzzle.hideHelp()">OK</button></div></div>';
};


puzzlepage.help = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="notouch"></div>';
};
