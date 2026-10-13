# TODO List alias Back Log :)
✅❗⚠️🅿️#️⃣*️⃣▶️☑️

## Vylepšení 
- ❗Geometrická validace natočení antén 
- ❗Odstranit optimalizaci počátečního úseku v pilot-robotour

## Zpracování dat ze Stromovky
- Prozkoumat data Lidaru pro účely Výpočtu kolizního kurzu
- Prozkoumat data Lidaru pro účely Výpočtu sjízdnosti

## Kamery a zpracování obrazu
- Zapojení natrénovaného modelu pro detekci cesty do pilota

## Detekce předmětů
- Vymyslet a zapojit model pro hledání objetů (dopravní kužel)



# Vyřešeno
- ✅ Zjistit možnosti regulace jasu na kamerovém chipu v omezené části chipu
  - ✅ dá se definovat oblast automatické expozice a případně ji ovlivňovat softwarově
- ✅ Prozkoumat data Kamer pro účely V-ODO 
  - ⚠️ obrazová data nejsou dost kvalitní pro V-ODO
- ⚠️ Aktivni hold podvozku. Udržet robota na pozici (implementace na hoverboardu)
- ☑️ Posílání odometrie, kroků z hall senzorů
- *️⃣ Implemtovat nastavení oblasti pro expozici kamer 
- Integrovat RTCM v3 do F9R a UM982
- Implementovat odesílání gps zpráv z UM982
- Nastavit F9R 
- Vyčítat IMU data z F9R
- Sloučit GPS Way points z uložených a vytvořit dlouhou trasu
- Vyrazit na kopec a jet podle GPS way pointů
- Porovnat kvalitu data
- Vytvořit službu pilot-manuál 
- Vyrazit do stromovky a nasnímat data
- Natrénovat nový model pro detekci cesty

# Zahozeno / Bezpředmětné 
- Test dead-reckoning módu F9R