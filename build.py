#!/usr/bin/python
# Compresses the core Blockly files into a single JavaScript file.
#
# Copyright 2012 Google Inc.
# http://code.google.com/p/blockly/
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import httplib, json, urllib, sys

filenames = [
    'blockly.js',
    'block.js',
    'block_svg.js',
    'comment.js',
    'connection.js',
    'contextmenu.js',
    'field.js',
    'field_dropdown.js',
    'field_label.js',
    'field_textinput.js',
    'flyout.js',
    'generator.js',
    'inject.js',
    'mutator.js',
    'names.js',
    'procedures.js',
    'scrollbar.js',
    'toolbox.js',
    'tooltip.js',
    'trashcan.js',
    'utils.js',
    'variables.js',
    'workspace.js',
    'xml.js']

target_filename = 'demos/blockly_compressed.js'

def file_lookup(name):
  if not name.startswith('Input_'):
    return '???'
  n = int(name[6:])
  return filenames[n]

# Define the parameters for the POST request.
params = [
    ('compilation_level', 'SIMPLE_OPTIMIZATIONS'),
    ('output_format', 'json'),
    ('output_info', 'compiled_code'),
    ('output_info', 'warnings'),
    ('output_info', 'errors'),
    ('output_info', 'statistics'),
  ]

# Read in all the source files.
for filename in filenames:
  f = open(filename)
  params.append(('js_code', ''.join(f.readlines())))
  f.close()

# Send the request to Google.
headers = { "Content-type": "application/x-www-form-urlencoded" }
conn = httplib.HTTPConnection('closure-compiler.appspot.com')
conn.request('POST', '/compile', urllib.urlencode(params), headers)
response = conn.getresponse()
json_str = response.read()
conn.close

# Parse the JSON response.
json_data = json.loads(json_str)

if json_data.has_key('errors'):
  errors = json_data['errors']
  for error in errors:
    print 'FATAL ERROR'
    print error['error']
    print '%s at line %d:' % (
        file_lookup(error['file']), error['lineno'])
    print error['line']
    print (' ' * error['charno']) + '^'
else:
  if json_data.has_key('warnings'):
    warnings = json_data['warnings']
    for warning in warnings:
      print 'WARNING'
      print warning['warning']
      print '%s at line %d:' % (
          file_lookup(warning['file']), warning['lineno'])
      print warning['line']
      print (' ' * warning['charno']) + '^'
    print

  code = json_data['compiledCode']

  stats = json_data['statistics']
  original_b = stats['originalSize']
  compressed_b = stats['compressedSize']
  if original_b > 0 and compressed_b > 0:
    f = open(target_filename, 'w')
    f.write(code)
    f.close()

    original_kb = int(original_b / 1024 + 0.5)
    compressed_kb = int(compressed_b / 1024 + 0.5)
    ratio = int(float(compressed_b) / float(original_b) * 100 + 0.5)
    print 'SUCCESS: ' + target_filename
    print 'Size changed from %d KB to %d KB (%d%%).' % (
        original_kb, compressed_kb, ratio)
  else:
    print 'UNKNOWN ERROR'
