// This file was automatically generated from template.soy.
// Please don't edit this file by hand.

if (typeof puzzlepage == 'undefined') { var puzzlepage = {}; }


puzzlepage.messages = function(opt_data, opt_ignored, opt_ijData) {
  return '<div id="MSG" style="display: none"><span id="country1">オーストラリア</span><span id="country1Flag">flag_au.png</span><span id="country1FlagHeight">50</span><span id="country1FlagWidth">100</span><span id="country1Language">英語</span><span id="country1City1">メルボルン</span><span id="country1City2">シドニー</span><span id="country1HelpUrl">http://ja.wikipedia.org/wiki/オーストラリア</span><span id="country2">ドイツ</span><span id="country2Flag">flag_de.png</span><span id="country2FlagHeight">60</span><span id="country2FlagWidth">100</span><span id="country2Language">ドイツ語</span><span id="country2City1">ベルリン</span><span id="country2City2">ミュンヘン</span><span id="country2HelpUrl">http://ja.wikipedia.org/wiki/ドイツ</span><span id="country3">中国</span><span id="country3Flag">flag_cn.png</span><span id="country3FlagHeight">66</span><span id="country3FlagWidth">100</span><span id="country3Language">中国語</span><span id="country3City1">北京</span><span id="country3City2">上海</span><span id="country3HelpUrl">http://ja.wikipedia.org/wiki/中華人民共和国</span><span id="country4">ブラジル</span><span id="country4Flag">flag_br.png</span><span id="country4FlagHeight">70</span><span id="country4FlagWidth">100</span><span id="country4Language">ポルトガル語</span><span id="country4City1">リオデジャネイロ</span><span id="country4City2">サンパウロ</span><span id="country4HelpUrl">http://ja.wikipedia.org/wiki/ブラジル</span><span id="flag">国旗:</span><span id="language">言語:</span><span id="languageChoose">選んでください...</span><span id="cities">都市:</span><span id="error0">完ぺきです!\n%1 個のブロックが全問正解です。</span><span id="error1">惜しい! 1 個のブロックが間違っています。</span><span id="error2">%1 個のブロックが間違っています。</span><span id="tryAgain">強調されているブロックが正しくありません。\nがんばってください。</span></div>';
};


puzzlepage.start = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<table id="header" width="100%"><tr><td valign="bottom"><h1><span id="title"><a href="../index.html">Blockly</a> : パズル</span></h1></td><td><select id="languageMenu" onchange="BlocklyApps.changeLanguage();"></select>&nbsp; &nbsp;<button id="helpButton" onclick="Puzzle.showHelp();">ヘルプ</button>&nbsp; &nbsp;<button id="checkButton" class="launch" onclick="Puzzle.checkAnswers();">答え合わせ</button></td></tr></table><script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="shadow"></div><div id="help"><div style="padding-bottom: 0.7ex">それぞれの国 (緑) について、国旗をつなげて、言語を選んで、都市を並べてください。</div><iframe src="help.html?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '" style="height: 200px; width: 100%; border: none;"></iframe><div class="farSide" style="padding: 1ex 3ex 0"><button id="okButton" onclick="Puzzle.hideHelp()">OK</button></div></div>';
};


puzzlepage.help = function(opt_data, opt_ignored, opt_ijData) {
  return puzzlepage.messages(null, null, opt_ijData) + '<script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script><div id="blockly"></div><div id="notouch"></div>';
};
