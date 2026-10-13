# Robotour 2026 - 19. září 2026 

**Mise:** Stromovka Praha (omezená část v okolí Planetária) - Dojet na místo zadané QR kódem.  
**Hlavní výzvy:** Detekce cesty, Localizace na mapě, Zpracování dat z Lidaru (traverzibilita a kolizní kurzy), Integrace uživatelského vstupu pro zadávání cíle, Vyhýbání se překážkám, Volba trasy, Hledání alternativní trasy, Dead Reconning pod mostem či v tunelu.

## Když se štěstí únaví ...

Během sběru dat pro trénování neuronové sítě pro detekci cest ve Stromovce došlo k pádu robota na IMU senzor, který odešel do věčných lovišť. 

Štěstí v neštěstí - IMU modul byl spíše podprůměrné kvality, ale zachraňoval to magnetický kompas, který se ukázal být na robotovi plném železa ne příliš spolehlivý. Stalo se to 14 dnů před soutěží a nebyl čas na nákup toho samého modulu z Číny. Místně nakoupený IMU modul nekomunikoval s I2C. Další nákup s IMU se vozil týden pro nedostatek místa v místě doručení mezi Plzní a Starým Kramolínem (dorazil v pátek dopoledne). Ale to už by na robotovi namontovaný původní GNSS modul F9R, který měl v sobě 6-osé IMU a službu jsem už měl napsanou. 

## Drama až do startu ...
Je středa před sobotní soutěží, neuronová síť pro detekci cesty je natrénovaná a robot má konečně nějakou IMU.  Čas na odladění IMU a fúze azimutu. 

### Aktuální stav: ###
- Detekce cesty - neuronová síť je natrénovaná, ale je třeba rozchodit Docker a Vision službu. 
- Localizace na mapě - testování z Dívky nese své ovoce, RTK FIX pod stromy + posílání GNSS polohy z duálního modulu je funkční.
- Zpracování dat z Lidaru (traverzibilita a kolizní kurzy) - lidar nevidí nic co je blíže jak 60cm ... ???
- Integrace uživatelského vstupu pro zadávání cíle - Vzít hlavu robota na dovolenou se vyplatilo ... služby Terminál i QR Scanner jsou připravené.
- Vyhýbání se překážkám - ... dream on ...
- Volba trasy - nová mapová služba ... koncept je jasný ...
- Hledání alternativní trasy, Dead Reconning pod mostem či v tunelu - {} "(prázná množina :)"

### Pomoc panika ... a zase štěstí? ###
Tady bych rád poděkoval své drahé polovičce, která vše zjenodušila na: "Soustřeď se na to, aby robot ujel jeden metr".

Zpětně se dá říct, že jich robot nakonec ujel něco kolem 9000 :)

## Nový plán ... Mission-Robotour
- Zadání cíle QR kódem - Odladění uživatelského vstupu pro zadávání cíle
- Nalezení cesty v offline mapě 
- Clone služby PILOT-WAIPOINS (ladění jízdy podle GPS souřadnic, už od Robotem rovně) a úprava na PILOT-ROBOTOUR.
- Když zbude čas, tak zkusit opravit na lidar (lepší bezpečnost) 

## Pátek odpoledne
Naprogramováno, otestováno ... Z nějakého důvodu se občas robot po zadání cíle rozjede nekotrolovaně dozadu ??? Ale je čas vyrazit do Stromovky, jsou to 3 hodiny cesty. Tak to bude oprava za jízdy. :)

Odpoledne ve Stromovce u Planetária je plno ... MRVK sedí na lavičce a ladí robota, o kousek dál nějaká rodinka s robotem. No nic, je čas na lepší mapu. 

V noci analyzuji GPS trace a vytvářím offline mapu pro robota. (... doplnit odkaz na https://unidroids.com/robot-platform/gnss-trace/gps-data-robotour-2026.zip ... ) Link obsahuje GPS trace z pátku, offline mapu i gnss data ze sobotní jízdy.

V datech je vidět přeskakování robota zleva doprava a zase zpátky. Díky za AI (v tomto případě Gemini Antigravity IDE), zkontrolovala GPS trace a doporučila úpravu fúze dat. 


## Sobota 

Někteří zkušení účastníci říkají, pořádně se vyspi. Tak jsem si dal 1,5 hodiny spánku :) Na start přicházíme o něco později, než jsme plánovali, je 9 hodin.

### Předkolo 9:00-10:00
Potkávám spoustu známých tváří. Martin si chce odšktrnout homologaci. Vše cajk. Na webu jsou QR kódy pro Home, Předkolo nakládka a vykládka. Je čas na zkušební jízdu.  
Robot dojede na nakládku, pak na vykládku, vše vypadá slibně, ... ale z nakládky hned vedle vzrostlého stromu co bych s těží obejmul ... se po zadání opakované nakládky robot otočí na místě a zastaví ... čekám na GPS polohu ... (na terminálu svítí přesnost polohy 0,7m) ... zkouším robota kousek poposunout, přerušit jízdu, nakonec se robot rozjede ... pokračuje na nakládku a zpět na vykládku ... a opět po zadání cíle (tentokrát Home) se otočí a zůstane stát ... čekám na GPS polohu ... (na terminálu svítí přesnost polohy 22mm ???) ... popošťuchování nepomhlo ... přerušuji jízdu a zadavám cílovou polohu znova ... robot se chytá a jede do servisní zóny. 

### 1.Kolo 10:00-11:00
Kam se podělo 30 minut mezi koli? ... No nic sedím na lavičce a deleguji problém na AI, tak po 10 minutách přemýšlení odpoví, že problém nebyla poloha, ale přesnost úhlu, který se na terminálu samozřejmně neukazuje :) 

Oprava limitu ze 6 stupňů na 15. V době kdy robot stál byla chyba natočení 6.1 stupně ... 

No nic je 10:30 všichni už jsou dávno na cestě. Jdeme na start, zadáváme místo nakládky pro 1. kolo. Robot jede, výrazně lépe než jsem očekával, bál jsem se, že bude ujíždět ke kraji a budeme se modlit, aby mu zůstalo jedno kolo na cestě, ale nic z toho nenastalo, teda prozatím ...

Nakládka proběhla jak po másle ... a teď místo vykládky, robot jede bez zaváhání, cesta vede pod svahem a algoritmus co jsem spíchnul funguje. Radost netrvala dlouho, zadání opakované nakládky, robot vyráží, jede rovně, ale ... postupně sjíždí z cesty ... až úplně sjel ... a první kolo končí ... asi chyba polohy??? ...  

Návrat do servisního zóny. Robot jede po vlastní ose bez viditelných problémů ...

### Pauza mezi koly

Nedostate spánku si vybírá své, sedím na židličce a odpočívám ... žádné myšlenky na analýzu, co se stalo ...

### 2.Kolo 11:30-12:30

Druhé kolo v plném proudu hned od začátku. Spouštíme robota v servisní zóně a jedeme směr nakládka, ale nejdeme po cestě, robot najíždí na trávník hned po vyjetí ... OK, konec druhého kola ... no co ... letos to asi 51 bodů z každého kola nebude ... 

S robotem vyrážíme jen tak a zadáváme nakládku, robot jede jak má, povídáme s Tomášem Píchou co přišel pomoct s organizací Martinovi (hlavní rozhodčí). Robot dojel na nakládku, zadáváme kód pro vykládku ... robot se otočí na místě a jede správným směrem ... po úzké cestě přijíždí favorit soutěže tým Helhest, robota zastavuji a uvolňuji cestu ... a pak ... telefonát ... Jaruška (moje žena) volá a říká, že robot sjel v semi zóně ... můžu opakovat druhé kolo ... ?!?

Robota tlačím zpět na start, přeci jen je už skoro 12 hodin. Jsme zpět na startu a zadáváme nakládku je 11:55 ... tentokrát robot nezaváhal ... jede seběvědomě středem cesty ... ostatní už dojezdili ... jsme na trase sami ... nakládka ... vykládka ... opakovaná nakládka ... vykládka ... a hajdy do servisní zóny ... robot ujel 1,4 km za 33:29s ... robot projel cílem  ve 12:27:05. Dvě minuty do limitu ... to bylo těsný ... Prvních 51 bodů 🎉


### Pauza na oběd a mezi koli

Eufórie z 51 jedna bodů dodává energii ... koukám na jiné roboty ... povídám si s kluky z Ultimo Minuto ...

Pak doráží objednaná pizza ... Martin si dal své vlastní kolo nakládky a vykládky ... dopravit 27 pizz pro všechny ... jen to vzal ještě po staru :)

### 3.kolo 14:00-15:00

Všichni vyráží na start a my též ... obnovuji stránku s QR kódem ... zadávám ho robotovi, ten vyráží jako by se nechumelilo ... první křižovatka a robot jede jinam než na nakládku ... ??? ... ano obovení stránky načetlo nový QR kód pro třetí kolo, ale čeho jsem si nevšiml bylo, že je to QR kód pro vykládku ... 

No nic, zastavuji robota a vracím se na start, předkládám problém rozhodčím ... s otázkou co dál ... rozhodčí povolují opakovaný start ... jupí :)

Je 14:15 a robotovi předkládám dvakrát zkontrolovaný QR kód ... ano je to 3.kolo ... (když o tom těď přemýšlím tak tam není napsaná "nakládka" u vykládky popis "vyklákda" je, ale už je to skoro měsíc ... tak kdo ví) ... robot vyráží na cestu ... po chvíli 5 minutách robot doráží do místa nakládky ... "naložit" ... a vzhůru do vykládky ...

Cesta na vyknakládku měla vsuvku v podobě protijeďoucího týmu MRVK ... robot neuměl nic jiného než zastavit ... což udělal na výbornou ... jen trochu komplikoval život MRVK ... kteří zvládli naprogramovat vyhybání se překážkám a když něměl dost místa tak si couvl ... (skvělá práce) ...

Robot viděl místo před sebou a trochu popojel, tím zmenšil místo pro manévr MRVK ... ten musel couvnutí opakovat ... to se stalo celkem třikrát ... na potřetí měl MRVK robot dost místa a robota objel ...

Pak už to byla klasika, vyložit, opakovaně naložit a vyložit a hajdy do servisní zóny ... robot dojel na čas, ujel 2 km za 40 minut. Dalších 51 bodů 🎉


### Pauza mezi koli

Tým MRVK vysílá zástupce, aby podal protest proti způsobu navigace robota. Kdy jsme jeli podle zřesněných mapových podkladů. Čímž se referovalo na rozhodnutí z roku 2009, kdy někdo provedl týdenní snímkování místa soutěže ...

Zpětně se dá říct, že jediným pochybením bylo, že jsem naskenovanou trasu nepublikoval na Open Street maps, což pravidla umožňují ... (sice nevím kdo by aktualizovanou mapu ze soboty 01:00 hodin v den soutěže použil) ... ale v principu ...

Béďa šéf Helhestu nad tím v té chvíli mávnul rukou, se slovi ... kdy šlo o nějaké price money ...

### 4.kolo 15:30-16:30

Nakládka a vykládka jsou daleko od sebe ... tým získal pozornost ... rozhodčí je šéf slovenského univerzitního týmu MRVK ... za námi jde nějaká skupinka ...

Ale kousek zpět, Helhest a my máme stejně bodů ... ale nechci se jim motat do cesty ... vím, že náš robot se neumí vyhnout a jen pasivně čeká než se cesta uvolní ...

Štěstí došlo? ... Helhest vyřáží první ... Když jsou na cestě zadávám místo nakládky ... a robot jede jinou cestou než Helhest ... hmmm ... Helhest zvolil delší cestu ... do místa nakládky přijíždíme dřív ... no nic ... zadáváme vykládku ... robot se otočí a jede po cestě zpět ... mezitím Helhest dojel na nakládku ... a robot k němu dojel a zastavil cca 60 cm od robota ... (všichni už věděli, že robot je pasivní co se týče překážek) ... takže jen čekal ... než Helhest objede ... 

Béďa (šéf Helhestu) měl obavu, že si není jist, tím, jestli jejich robot náhodou nesmete při otáčení jiného robota ... (používal trochji jiná slova, ale snad jsem pochopil správně co myslel tím, když se ptal jestli si chi stavět nového robota) ...

Slovo dalo slovo a tým Helhest zadává místo vykládky ... výsledek ... jejich robot má snahu se pomalu otočit ... nejspíš protože má překážku moc blízko ... ale konstrukce robota týmu Helhest neumoňuje hladké otočení ... otáčí se smykem ... v místě kde se chtěl otočit je drsný asfalt a robot se nehnul ...

Následovala slovní snaha podle nějakého pravidla, tým nás blokuje, odstraňtě robota, ... náš robot jel zatím co váš stál ... no prostě pointless ...

Výsledek ... po několika minutách přijíždí další tými ... ISTROBOTICTS a COGITO ... ISTROBOTICS se vyhybá skrumáži, provádí nakládku a odjíždí ... ale COGITO s přesnou GPS neomilně míří do skumáže ...

Naštěstí přichází i hlavní rozhodčí a spletenec rozmotává. Helhest odjíždí první a robot po něm ...

Drama až do konce ... robot jede svým tempem ... následuje virtuální GPS čáru ... no prostě neuvěřitelné ... daleko za nejdivočejšími sny ...

No nic zpět na trať ... robot se blíží k místu vykládky, kde Helhest zrovna vyložil ... a co čert nechtěl ... přístupová cesta k místu vykládky je široká tak na jednoho robota ... metr dvacet ... 

Načasování jak z učebnice dopravního inspektorátu ... robot zablokoval výjezd z cesty ... a protože vidí překážku zastavuje ... Helhest už byl v pohybu a tak se robotovi vyhne, ale ...

Sjíždí z cesty na travnatý povrch ... všichni bez mrknutí oka vědí ... to se nikdy nestalo :)

Robot se rozjel ... na straně v místě nakladky je zaparkovaný robot týmu ISTROBOTICTS ... a vypadá to na nějaké problémy ... robot se jim rozjel tak prudce, že mu podjela kola a padl na záda ... upadla kamerka a GPS anténa ... opravdu škoda ...

No nic jsme v místě vykládky, tentokrát vyndáváme soudek, neb jsme byli upozorněni, že je třeba jezdit i bez něj ... tady bych rád poděkoval šéfovi slovenského týmu MRVK za to, že soudek vzal ... asi bych nevědel co dřív, kdy robot zpanikařil a já měl v ruce 5l soudek piva ...

Je 16:06 ... už jsme více jak půlhodiny na cestě ... no nic zkusíme to ... Vyrážíme na opakovanou nakládku ... přijíždíme k nakládce a Helhest nikde, čekal jsem, že nás budou míjet ... ale nebyli tam ... asi se rozhodli, že opakovanou nakládku nebudou dělat a jeli rovnou do servisní oblasti ...

Jsme na nakládce, nakládáme soudek a vyrážíme na vykládku ... je 16:17 ... robot se drží trasy, s rozhodčím si povídáme o tom jak je robot postavený a co funguje a kde hardware překonává software ... přijíždíme k vykládce ... je 16:28 ... 

ISTROBOTICTS stále na místě opravuje robota ... (škoda, že jsem nebyl víc při smyslech a nenabídl jim pomoc, mohl jsem něco vzít ...) 

Zadávám QR kód pro servisní zónu ... robot doráží do cíle 16:39 ... 

Trasa byla 2,4 km a celkem to trvalo 1:07 minut ... 

## Sečteno podtrženo 

Výsledek všech kol 164 bodů z 204 možných 🎉 

Když zvážím, že robot neměl všechny plánované vylepšení ... Detekce cesty, Zpracování dat z Lidaru (traverzibilita a kolizní kurzy), Aktivní vyhýbání se překážkám, Hledání alternativní trasy, Dead Reconning pod mostem či v tunelu ...

Tak se zdá, že největším problémem delivery robota je jeho lokalizace v mapě, což pro Stromovku dokázali moderní GNSS moduly a antény ... 

A díky všem 131 satelitům (50 čínských, 28 Evropských, 25 Ruských, 28 Amerických) se robotovy podařilo lokalizovat s přesností cca 3 cm a jet opakovaně podél virtuální trasy. 

Skvělá práce všichni 🎉 

## Techická analýza logů a hledání chyb


## Prezentace 


