// This file was automatically generated from template.soy.
// Please don't edit this file by hand.

if (typeof mazepage == 'undefined') { var mazepage = {}; }


mazepage.start = function(opt_data, opt_ignored, opt_ijData) {
  var output = '<div id="MSG" style="display: none"><span id="moveForward">иди напред</span><span id="turnLeft">окрени се лево</span><span id="turnRight">окрени се десно</span><span id="doCode">ради</span><span id="elseCode">у супротном</span><span id="pathAhead">ако постоји пут напред</span><span id="pathLeft">ако постоји пут лово</span><span id="pathRight">ако постоји пут десно</span><span id="repeatUntil">понављај док не</span><span id="moveForwardTooltip">Помера Штипаљка једну позицију напред.</span><span id="turnTooltip">Окреће Штипаљка лево или десно за 90 степени.</span><span id="ifTooltip">Ако постоји пут у датом правцу, онда уради следеће.</span><span id="ifelseTooltip">Ако постоји пут у датом правцу, \\nонда изврши прву групу акција. \\nУ супротном, уради другу групу \\nакција. </span><span id="whileTooltip">Понављај следеће акције док не комплетираш задатак.</span><span id="capacity0">Број блокова који ти је остао је 0.</span><span id="capacity1">Број блокова који ти је остао је 1.</span><span id="capacity2">Број блокова који ти је остао је %1.</span><span id="nextLevel">Честитамо! Да ли сте спремни да пређете на ниво %1?</span><span id="finalLevel">Честитамо! Решили сте последњи ниво.</span><span id="oneTopBlock">На овом нивоу, потребно је да поређаш све блокове у бели радни простор.</span></div><div id="COMMON_MSG" style="display: none"><span id="httpRequestError">Настао је проблем при извршењу траженог.</span><span id="linkAlert">Делите своје блокове употребом ове везе:\n\n%1</span><span id="hashError">Жао нам је, \'%1\' не одговара ни једној сачуваној Блокли датотеци.</span><span id="xmlError">Не могу да учитам датотеку коју сте сачували.  Можда је направљена другом верзијом Блоклија?</span></div><table width="100%" height="100%"><tr height="50"><td colspan=3><div style="display: none;"><select id="languageMenu" onchange="BlocklyApps.changeLanguage();"></select> &nbsp; <button id="pegmanButton" onmousedown="Maze.showPegmanMenu();"><img src="../../media/1x1.gif"><span>&#x25BE;</span></button></div><h1><span id="title"><a href="../index.html">Blockly (Блоклијев)</a> : Лавиринт</span> &nbsp; ';
  for (var i118 = 1; i118 < 11; i118++) {
    output += ' ' + ((i118 == opt_ijData.level) ? '<span class="tab" id="selected">' + soy.$$escapeHtml(i118) + '</span>' : (i118 < opt_ijData.level) ? '<a class="tab previous" href="?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '&level=' + soy.$$escapeHtml(i118) + '&skin=' + soy.$$escapeHtml(opt_ijData.skin) + '">' + soy.$$escapeHtml(i118) + '</a>' : '<a class="tab" href="?lang=' + soy.$$escapeHtml(opt_ijData.lang) + '&level=' + soy.$$escapeHtml(i118) + '&skin=' + soy.$$escapeHtml(opt_ijData.skin) + '">' + soy.$$escapeHtml(i118) + '</a>');
  }
  output += '</h1></td></tr><tr><td width="400" valign="top"><div style="position: relative"><div id="hintBubble"><div id="hint">';
  switch (opt_ijData.level) {
    case 1:
      output += 'Програм је низ исказа. Наређај неколико \'иди напред\' блокова да би ми помогао да достигнем циљ.';
      break;
    case 2:
      output += 'Који низ корака треба да се предузме да би се прошао пут?';
      break;
    case 3:
      output += 'Рачунари имају ограничену меморију. Дођи до краја пута употребивши само два блока. Искористи \'понављај\' да би извршио блок бише пута.';
      break;
    case 4:
      output += 'Достигни циљ употребом само пет блокова.';
      break;
    case 5:
      output += 'Pegman will have to turn left when he cannot go straight.';
      break;
    case 6:
      output += '\'ако\' блок ће урадити нешто једино ако је услов испуњен. Покушај окрет у лево ако постоји пут са леве стране.';
      break;
    case 7:
      output += 'Овај лавиринт изгледа сложеније него предходни, али није.';
      break;
    case 8:
      output += 'Можете да употребите више \'ако\' блокова.';
      break;
    case 9:
      output += 'Ако-у-супротмом блок урадиће једно или друго.';
      break;
    case 10:
      output += 'Можеш ли да решиш овај сложени лавиринт? Покшај да се крећеш уз леви зид.';
      break;
  }
  output += '</div></div><svg xmlns="http://www.w3.org/2000/svg" version="1.1" id="svgMaze" width="400px" height="450px"><g id="look"><path d="M 0,-15 a 15 15 0 0 1 15 15" /><path d="M 0,-35 a 35 35 0 0 1 35 35" /><path d="M 0,-55 a 55 55 0 0 1 55 55" /></g></svg><div id="capacityBubble"><div id="capacity"></div></div></div><table width="100%"><tr><td style="width: 190px; text-align: center; vertical-align: top;"><button title="Погледај генерисани JavaScript код." onclick="BlocklyApps.showCode();"><img src="../../media/1x1.gif" class="code icon21"></button><button id="linkButton" title="Сачувај и повежи са блоковима. " onclick="BlocklyStorage.link();"><img src="../../media/1x1.gif" class="link icon21"></button></td><td><button id="runButton" class="launch" onclick="Maze.runButtonClick();"><img src="../../media/1x1.gif" class="run icon21"> Покрени програм</button><button id="resetButton" class="launch" onclick="Maze.resetButtonClick();" style="display: none"><img src="../../media/1x1.gif" class="stop icon21"> Поново постави</button></td></tr></table></td><td width=5></td><td valign="top"><script type="text/javascript" src="../../blockly_compressed.js"><\/script><script type="text/javascript" src="../../javascript_compressed.js"><\/script><script type="text/javascript" src="../../' + soy.$$escapeHtml(opt_ijData.langSrc) + '"><\/script><script type="text/javascript" src="blocks.js"><\/script>' + mazepage.toolbox(null, null, opt_ijData) + '<div id="blockly"></div></td></tr></table><div id="pegmanMenu"></div>';
  return output;
};


mazepage.toolbox = function(opt_data, opt_ignored, opt_ijData) {
  return '<xml id="toolbox" style="display: none"><block type="maze_moveForward"></block><block type="maze_turn"><title name="DIR">turnLeft</title></block><block type="maze_turn"><title name="DIR">turnRight</title></block>' + ((opt_ijData.level > 2) ? '<block type="maze_forever"></block>' + ((opt_ijData.level == 6) ? '<block type="maze_if"><title name="DIR">isPathLeft</title></block>' : (opt_ijData.level > 6) ? '<block type="maze_if"></block>' + ((opt_ijData.level > 8) ? '<block type="maze_ifElse"></block>' : '') : '') : '') + '</xml>';
};
