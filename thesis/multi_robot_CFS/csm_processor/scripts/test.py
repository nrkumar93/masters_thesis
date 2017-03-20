#!/usr/bin/env python

import subprocess

def main():
	cmd = 'roscd csm_processor && cd ../../../../../devel/lib/csm_processor'
	subprocess.call(cmd, shell=True)
	return 0

if __name__ == '__main__':
	main()

