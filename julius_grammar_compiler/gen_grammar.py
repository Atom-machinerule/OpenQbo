#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (GPLv2 License)
#
# Copyright (c) 2012 TheCorpora SL
#
# This program is free software; you can redistribute it and/or 
# modify it under the terms of the GNU General Public License as 
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License 
# along with this program; if not, write to the Free Software 
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
# MA 02110-1301, USA.
#
# Authors: Sergio Merino <s.merino@openqbo.com>

import shlex, subprocess, shutil
import sys
import re
import random

#This function create the voca file
LMdirectory="/opt/qbo/ros_stacks/qbo_apps/qbo_listen/config/LM/"
AMdirectory="/opt/qbo/ros_stacks/qbo_apps/qbo_listen/config/AM/"
def getRandomwords(wFile,num):
    f=open(wFile,"r")
    count=0
    numlist=[]
    wordlist=[]
    for line in f:
        count=count+1
    for i in range(1, num+1): 
        numlist.append(random.randint(1, count))
    numlist=sorted(numlist)
    f.close()
    f=open(wFile,"r")
    count=0
    for line in f:
        count=count+1
        if count==numlist[0]:
            separated=line.split()
            wordlist.append(separated[0])
            numlist.remove(count)
            if len(numlist)==0:
                break
    return wordlist

def addwordtypelabel(word, filename, label):
    tmpfile="/tmp/addwordtmpfile"
    f1=open(filename,"r")
    f2=open(tmpfile,"w")
    added=False
    inlabel=False
    repeated=False
    label=label.lower()
    word=word.lower()
    for line in f1:
        auxline=line.strip()
        if added==False and auxline==label:
            inlabel=True
        elif inlabel==True:
            if auxline==word:
                repeated=True    
                added=True 
                break
            elif auxline!="" and auxline[0]=="[":
                added=True
                inlabel=False
                f2.write(word.lower())
                f2.write("\n")
        f2.write(line)
    if added==False:
        f2.write(label+"\n")
        f2.write(word+"\n")
    if repeated==False:
        f1.close()
        f2.close()
        copyfiles(tmpfile,filename)

def getlinephonem(word,phonemfile):
    f=open(phonemfile,"r")
    solution=""
    found=0
    for line in f:
#        line=line.upper()
        separated=line.split()
        if separated[0] == word:
                found=1
                if separated[1][0]=="[":
                    separated.pop(1)
                newline=""
                for i in separated:
                        newline=newline+str(i)+" "
                solution=solution+newline+"\n"
       # elif separated[0] > word and str(separated[0]) <> "<s>" and str(separated[0]) <> "</s>" :
       #         if found==0:
       #                 print "ERROR: The word "+str(word)+" wasnt found"
       #         break
    if found==0:
        solution=-1
    f.close()
    return solution

def verrors(configfile, phonemfile):
    f=open(configfile,"r")
    result=[]
    for line in f:
        line=line.strip()
        line=line.upper()
        if len(line)<>0 and line[0]<>"[":
                separated=line.split()
                if len(separated)==1:
                        word=separated[0]
                        if word[0]<>"{":
                            error=getlinephonem(word,phonemfile)
                            if error==-1:
                                result.append(word)
                else:
                    for word in separated:
                        if word[0]<>"{":
                            error=getlinephonem(word,phonemfile)
                            if error==-1:
                                result.append(word)
    if result==[]:
        return 0
    else:
        return result


def reviewphon(phonlist,tiedfile):
    f1=open(tiedfile,"r")
    for line in f1:
        line=line.strip()
        separated=line.split()
        try:
            phonlist.remove(separated[0])
        except:
            i=1
    f1.close()
    return phonlist


def perrors(vocafile, tiedlist):
    f1=open(vocafile,"r")
    newlist=[]
    for line in f1:
        line=line.strip()
        if line<>"" and line[0]<>"%":
            separated=line.split()
            separated.pop(0)
            for i in range(len(separated)):
                if i==len(separated)-2:
                    if len(separated)==2:
                        if newlist.count(str(separated[i])+"+"+str(separated[i+1]))==0:
                            newlist.append(str(separated[i])+"+"+str(separated[i+1]))
                        else:
                            if newlist.count(str(separated[i])+"-"+str(separated[i+1]))==0:
                                newlist.append(str(separated[i])+"-"+str(separated[i+1]))
                if i<len(separated)-2:
                    if newlist.count(str(separated[i])+"-"+str(separated[i+1])+"+"+str(separated[i+2]))==0:
                        newlist.append(str(separated[i])+"-"+str(separated[i+1])+"+"+str(separated[i+2]))
    f1.close()
    result=reviewphon(newlist,tiedlist)
    if len(result)==0:
        return 0
    else:
        return result


def createvoca(configfile, phonemfile, dstfile):
    f=open(configfile,"r")
    f2 = open(dstfile+".voca","w")
    f3 = open(dstfile+".grammar","w")
    f2.write("%NS_B\n")
    f2.write("<s>        sil\n")
    f2.write("%NS_E\n")
    f2.write("</s>        sil\n")
    f3.write("S: NS_B SENTENCES NS_E\n")
    voca=""
    gram=""
    alonewords=""
    wordcount=0
    wordlist=[]
    for line in f:
        line=line.strip()
        line=line.upper()
        if len(line)<>0 and line[0]=="[":
            sections=line.split()
            section=str(sections[0])
            section=re.sub("[\[\]]", "", section)
        elif len(line)<>0:
            separated=line.split()
            if len(separated)==1:
                word=separated[0]
                if word[0]<>"{":
                    try:
                        pos=wordlist.index(word)
                        gram=gram+section+": W"+str(pos)+"\n"
                        f3.write(section+": W"+str(pos)+"\n")
                    except:
                        wordlist.append(word)
                        voca=voca+"%W"+str(wordcount)+"\n"
                        voca=voca+getlinephonem(word,phonemfile)+"\n"
                        f2.write("%W"+str(wordcount)+"\n")
                        f2.write(getlinephonem(word,phonemfile)+"\n")
                        wordcount+=1
                        pos=wordlist.index(word)
                        gram=gram+section+": W"+str(pos)+"\n"
                        f3.write(section+": W"+str(pos)+"\n")
                else:
                    word=re.sub("[{}]","",word)
                    gram=gram+section+": "+str(word)+"\n"
                    f3.write(section+": "+str(word)+"\n")
            elif len(separated)>1:
                gram=gram+section+":"
                f3.write(section+":")
                for word in separated:
                    if word[0]<>"{":
                        try:
                            pos=wordlist.index(word)
                            gram=gram+" W"+str(pos)
                            f3.write(" W"+str(pos))
                        except:
                            wordlist.append(word)
                            voca=voca+"%W"+str(wordcount)+"\n"
                            voca=voca+getlinephonem(word,phonemfile)+"\n"
                            f2.write("%W"+str(wordcount)+"\n")
                            f2.write(getlinephonem(word,phonemfile)+"\n")
                            wordcount+=1
                            pos=wordlist.index(word)
                            gram=gram+" W"+str(pos)
                            f3.write(" W"+str(pos))
                    else:
                        word=re.sub("[{}]","",word)
                        gram=gram +" "+ word
                        f3.write(" "+ word)
                gram=gram+"\n"
                f3.write("\n")

    #print "VOCA:"
    #print voca
    #print "GRAM:"
    #print gram
    f2.close()
    f3.close()

def copyfiles(filename1, filename2):
    shutil.copyfile(filename1, filename2)

def helpmessage():
    print ("Version 2.02")
    print ("Usage: gengram sentencefilename phonemsfilename tiedlist destfilename")
    print ("\tConfig file: Name of the file with the sentences")
    print ("\tphonemsfilename: Name of the phonems file")
    print ("\ttiedlist: HMMlist file")
    print ("\tdestfilename: Name of the gramar, voca, dict to be created")
    print ("Config file format:")
    print ("\tUse \[\] to create labels.")
    print ("\tUse \{\} to reference labels.")
    print ("\tLabel 'sentences' is always needed.")
    print ("\tIn a label you can referer another label.")
    print ("\tDo not make loops.")
    print ("\tDo not add silences, it will be added automatically.")
    print ("\tExample:")
    print ("\t\t[name]")
    print ("\t\tJohn")
    print ("\t\tRambo")
    print ("\t\t[objects]")
    print ("\t\t{desktop_objects}")
    print ("\t\t{other_objects}")
    print ("\t\tnewspaper")
    print ("\t\t[sentences]")
    print ("\t\twhat is this")
    print ("\t\tthis is a {object}")
    print ("\t\tmy name is {name}")
    print ("\t\t[desktop_objects]")
    print ("\t\tpen")
    print ("\t\tscreen")
    print ("\t\t[other_objects]")
    print ("\t\tmask")
    print ("\t\tnewspaper")
    print ("phonemsfile 2 possible format:")
    print ("\tANDS            ae n d z")
    print ("\tANDUILLE                ae n d uw iy")
    print ("")
    print ("\tANDS   [ANDS]         ae n d z")
    print ("\tANDUILLE   [ANDUILLE]             ae n d uw iy")


def compilegrammar(grammar,lang):
    sentencefile=LMdirectory+lang+"/"+grammar+"/sentences.conf"
    phonemfile=AMdirectory+lang+"/"+"phonems"
    dstfiles=LMdirectory+lang+"/"+grammar+"/"+grammar
    createvoca(sentencefile,phonemfile,dstfiles)
    subprocess.call(["mkdfa.pl", dstfiles])
    
def main(argv):
    if len(argv)==1 and argv[0]=="-v" or argv[0]=="-V" :
        helpmessage()

    elif len(argv)<>4 :
        helpmessage()

    else:
        tmpfiles="/tmp/compilegramm"
        tmpvoca=tmpfiles+".voca"
        tmpgramm=tmpfiles+".grammar"
        sentencefile=argv[0]
        phonemfile=argv[1]
        tiedlist=argv[2]
        destfiles=argv[3]
        print "Looking for vocabulary inconsistencies..."
        vocaerrors=verrors(sentencefile,phonemfile)
        if vocaerrors==0:
            print "All vocabulary correct."
            print "Creating temporal vocabulary and gramma files..."
            createvoca(sentencefile,phonemfile,tmpfiles)
            print "Checking the phonems.."
            phonemserr=perrors(tmpvoca,tiedlist)
            if phonemserr==0:
                print "Creating final voca and grammar files..."
                copyfiles(tmpvoca,destfiles+".voca")
                copyfiles(tmpgramm,destfiles+".grammar")
                print "Compiling..."
                subprocess.call(["mkdfa.pl", destfiles])
                sys.exit(1)
            else:
                print "ERROR: Not all phonemsi tiedlist"
                print phonemserr
                return phonemserr
        else:
            print "ERROR: Not all words in diccionary"
            print vocaerrors
            return vocaerrors


if __name__ == "__main__":
        main(sys.argv[1:])

