import argparse
import json
import re
import os

titlere = re.compile('^\\s*#+\\s*([^#]+)\\s*#+') 
copyre = re.compile('.*&copy;\\s+([0-9]+)\\s+Jason')
def find_title(cells):
    for c in cells:
        if c['cell_type'] == 'markdown':
            m = titlere.match(c['source'][0])
            if m:
                return m.group(1)

def find_copyright_year(cells):
    for c in cells[::-1]:
        if c['cell_type'] == 'markdown':
            for s in c['source']:
                m = copyre.match(s)
                if m:
                    return m.group(1)
    print "no copyright found"

def get_code_cells(orig_cells):
    cells = [c for c in orig_cells if c['cell_type'] == 'code']
    istop = None
    stopre = re.compile('^#:\\s+stop')
    for i in xrange(len(cells)):
        if not cells[i]['input']:
            continue
        if stopre.match(cells[i]['input'][0]):
            istop = i
            print '#: stop found at %d' % istop
            break
    return cells[:istop]

def doit(infile, outfile, n):
    with open(infile) as f:
        j = json.load(f)
    orig_cells = j['worksheets'][0]['cells']
    cells = get_code_cells(orig_cells)
    
    if not cells:
        return False
    title = find_title(orig_cells)
    copy_year = find_copyright_year(orig_cells)
    url = 'http://www.embeddedrelated.com/showarticle/%d.php' % n
    header = '''
# %s #

IPython Notebook code &mdash; the [original article](%s) is located at
%s

&copy; %s Jason M Sachs

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
''' % (title, url, url, copy_year)
    cells[:0] = [{'cell_type': 'markdown', 'metadata':{},'source':
        header.splitlines()
        }]        
    j['worksheets'][0]['cells'] = cells
    with open(outfile,'w') as f:
        json.dump(j, f)
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert ipython nb to strip out markdown')
    parser.add_argument('indir',help='input directory')
    parser.add_argument('outdir',help='output directory')
    args = parser.parse_args()
    filesre = re.compile('^(\d+)-.*\\.ipynb$')
    files = [f for f in os.listdir(args.indir) if filesre.match(f)]
    for file in files:
        m = filesre.match(file)
        n = int(m.group(1))
        success = doit(infile=os.path.join(args.indir, file),
             outfile=os.path.join(args.outdir, file),
             n=n)
        print '*' if success else ' ', n, file
