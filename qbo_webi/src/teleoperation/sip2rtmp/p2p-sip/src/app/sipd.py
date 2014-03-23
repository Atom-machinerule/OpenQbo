# Copyright (c) 2008, Kundan Singh. All rights reserved. See LICENSING for details.
# Copyright (c) 2012 TheCorpora SL
'''
Implements a SIP server including the following functions:
1. SIP registration, proxy and redirect server
2. Two-stage scalable SIP server farm
3. Master-slave reliable SIP server farm

The high level script in this module controls the behavior of an incoming call similar to SIP Express Router (SER) config file. 
An examples from SER config file is at http://lists.iptel.org/pipermail/serusers/2004-December/013690.html
'''
from datetime import datetime
from app import sipapi
from std import rfc3261 

_debug = False  # To debug: enable this if importing as module, otherwise use option -d command line option if running as application
HP = lambda x: (x.partition(':')[0], int(x.partition(':')[2])) if ':' in x else (x, 5060) # convert "ip:port" to ("ip", port) and "ip" to ("ip", 5060)
HPS= lambda x: '%s:%d'%(x[0], x[1]) if x[1] != 0 and x[1] != 5060 and x[1] is not None else x[0] # convert ("ip", port) to "ip:port" or "ip"

_specialUser = ""
_botUser = ""

if __name__ == '__main__': # parse command line options, and set the high level properties
    from optparse import OptionParser, OptionGroup
    parser = OptionParser()
    parser.add_option('-d', '--verbose',   dest='verbose', default=False, action='store_true', help='enable debug for all modules')
    group1 = OptionGroup(parser, 'Registrar', 'Use these options to specify registrar options such as listening address and hosted domains. A hosted domain is the host portion of the URI for which this registrar will accept registrations. Any domain that is not hosted here is treated as foreign domain.')
    group1.add_option('-l', '--local',     dest='local',   default='0.0.0.0:5060', metavar='HOST:PORT', help='local listening HOST:PORT. Default is "0.0.0.0:5060"')
    group1.add_option('-r', '--domain',    dest='domain',  default=[], action='append',metavar='DOMAIN', help='restrict hosted domain, e.g., example.net. This option can appear multiple times. If the option is not specified, then it can host any domain.')

    group1.add_option('-u', '--specialuser',  dest='specialuser',   default='webi', metavar='', help='security issue')

    group1.add_option('-b', '--botUserName',  dest='botUserName',   default='qbobot', metavar='', help='security issue')

    parser.add_option_group(group1)

	

    group2 = OptionGroup(parser, 'Failover and Load Sharing', 'Use these options to specify multiple primary and secondary SIP servers, by using multiple -p and -s options. The server farm automatically uses two-stage architecture if there are multiple primary servers. Each instance of the server in the farm must use the same ordered list of server options.')
    group2.add_option('-p', '--primary',   dest='primary', default=[], action='append', metavar='HOST:PORT',  help='primary server HOST:PORT. This option can appear multiple times.')
    group2.add_option('-s', '--secondary', dest='secondary',default=[], action="append",metavar='HOST:PORT',  help='secondary server HOST:PORT. This option can appear multiple times.')
    parser.add_option_group(group2)

    (options, args) = parser.parse_args()
     
    
    if options.verbose: _debug = sipapi._debug = rfc3261._debug = True 


    _specialUser = options.specialuser
    _botUser = options.botUserName

    print "id y pwd de bot y user"
    print _specialUser
    print _botUser

    # create the proxy and registrar agent
    sipaddr = HP(options.local)
    agent = sipapi.Agent(sipaddr=sipaddr, transports=('udp',))
                

    agent.domain = options.domain                  # list of supported local domains for registrar.
    agent.location = sipapi.Location()             # storage for contact locations.
    # agent.subscriber = sipapi.Subscriber()       # list of subscribers. Enable this to allow only registered subscribers.
    agent.index, agent.primary, agent.secondary=0, [HP(x) for x in options.primary], [HP(x) for x in options.secondary] # failover and load sharing
    if agent.primary or agent.secondary:
        local = sipaddr if sipaddr[0] != '0.0.0.0' else ('localhost', sipaddr[1])
        if agent.primary and local in agent.primary: agent.index = agent.primary.index(local)+1
        elif agent.secondary and local in agent.secondary: agent.index = -agent.secondary.index(local)-1
        else: raise ValueError('local sipaddr %s argument must exist in primary or secondary if primary or secondary are specified'%(sipaddr))
    agent.start()  
    
def route(event):



    if event['From'].value.uri.user != _specialUser and event['From'].value.uri.user != _botUser :
         print '\033[1;38m  Unauthorized Access for: '+event['From'].value.uri.user+' action=['+event.method+'] \033[1;m'
         return event.action.reject(401, 'Unauthorized Access')


    if event['From'].value.uri.user == _botUser and event.method =="BYE":
	print '\033[1;38m  Bot tried to end the call, but we do not permit it \033[1;m'
        return 

    print ""
    print "USER: "+_specialUser
    print "BOT: "+_botUser


    print datetime.now()
    print "----- who  : "+event['From'].value.uri.user
    print "----- event: "+event.method
    print ""



    '''The main routing method for the server agent.'''
    # sanity check section
    if event['Max-Forwards'] and int(event['Max-Forwards'].value) <= 0: return event.action.reject(483, 'Too many hops')
    if len(str(event)) > 8192: return event.action.reject(513, 'Message overflow')
    # this is used by sipsak to monitor the health of server
    if event['From'].value.uri.user == 'sipsak' and event.method == 'OPTIONS' and not event.uri.user: return event.action.accept()
    # route header processing
    if event['Route']:
        event.location = event.first('Route').value.uri
        return event.action.proxy(recordRoute=(event.method=='INVITE'))
    # failover and load sharing section
    if event.agent.index > 0 and event.uri.user is not None: # if we are first stage proxy, proxy to second stage if needed
        index = hash(event.uri.user) % len(event.agent.primary)
        if len(event.agent.primary) > 1 and (index+1) != event.agent.index: # in the two-stage server farm, not for us
            event.location, dest = event.uri.dup(), event.agent.primary[index]
            event.location.host, event.location.port = dest
            return event.action.proxy(recordRoute=False)
    # registration section
    if event.method == 'REGISTER' or event.method == 'PUBLISH':
	

        if event.agent.domain and HPS(event['From'].value.uri.hostPort) not in event.agent.domain: return event.action.reject(403, 'Unknown Domain')
        if event.To.value.uri != event.From.value.uri: return event.action.reject(401, 'Unauthorized Third-party Registration')

        if hasattr(event.agent, 'subscriber'): # authenticate if subscriber table is present
	    print "hola?"
            auth = event.agent.subscriber.authenticate(event, realm='39peers.net')
	    print "------------ "+str(auth)
            if auth == 404: return event.action.reject(404, 'Not Found')
            elif auth == 401 or auth == 0: return event.action.challenge(realm='localhost') 
            elif auth != 200: return event.action.reject(500, 'Internal Server Error in authentication')




	print ":::::::::: ::::::::: ::::::"

        if not event.agent.location.save(msg=event, uri=str(event.To.value.uri).lower()): return event.action.reject(500, 'Internal Server Error in Location Service')
        return event.action.accept(contacts=event.agent.location.locate(str(event.To.value.uri).lower()))
    
    # whether the original request had Route header to this server?
    try: had_lr = event.had_lr
    except AttributeError: had_lr = False
    
    # open relay section
    if not had_lr and event.method == 'INVITE':
        if event.agent.domain and HPS(event['From'].value.uri.hostPort) not in event.agent.domain and HPS(event.uri.hostPort) not in event.agent.domain:
            return event.action.reject(403, 'Please register to use our service')
    else:
        if event.agent.domain and HPS(event.uri) not in event.agent.domain or not event.agent.domain and not event.ua.isLocal(event.uri): 
            if _debug: print 'proxying non-invite non-local request'
            event.location = event.uri
            return event.action.proxy()
    event.location = map(lambda x: x.value.uri, event.agent.location.locate(str(event.uri).lower()))
    if _debug: print 'locate returned', event.location
    return event.action.proxy(recordRoute=(event.method=='INVITE'))

if __name__ == '__main__': 
    agent.attach('incoming', route)
    sipapi.run()  # the loop to process the SIP listening point

