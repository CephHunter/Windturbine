README FILE FOR ARDUINO COMMUNICATION PROTOCOL - ONTWERPPROJECT 2017-2018
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------


//--- 1 ---//
Voor het ontwerpproject willen we alle windmolens tegelijk kunnen monitoren vanaf 1 webpagina/programma. Hiervoor hebben
we een bordje voor het draadloos doorsturen van data en de bijhorende code ontwikkeld. Om alles goed te laten werken moet
je de volgende dingen doen:

- Hou de pinnen 0 en 1 van je arduino vrij. Dit zijn de TX en RX pinnen die we gebruiken om data door te sturen.
- plaats/kopieer de volgende files in dezelfde map als je finale sketch: IP.c ; IP.h ; IPControl.c ; IPControl.h
- kopieer de code van de "communication" sketch in je eigen sketch. Deze code zal er voor zorgen dat de data correct 
  doorgestuurd en ontvangen wordt via het externe bordje.

//--- 2 ---//
Bespreking van de code.

	//--- 2.1 ---//
	Om alles te doen werken, moet je een aantal variabelen aanmaken voor je setup(). 
	
	- Socket_t connection: de verbinding tussen de zender en ontvanger
	- char stream[64]: een array van characters die als buffer dient om berichten tijdelijk in op te slaan
	- char previoustime: houdt de tijd bij zodat er om de 5 seconden data gestuurd kan worden.
	- void UART_receive() en void UART_Send(char* data, uint8_t len): declaratie van de functies om de ontvangen en sturen.

	//--- 2.2 ---//
	Voor het doorsturen van de data moet eerst de verbinding tot stand gebracht worden. Dit doe je in de setup() van je code.
	Allereerst start je de seriële communicatie op 9600 Baud. Vervolgens ga je de IPControl_setup instellen, hierbij geef je
	als eerste argument je ID in, dit is je groepsnummer. Het 2e argument van de functie is de functie die voor het verzenden 
	van de data zal zorgen, hier UART_Send. Deze functie moet onderaan je sketch staan zoals in de voorbeeldsketch "communication".

	//--- 2.3 ---//
	Om een veilige werking van de windmolen te verzekeren, moet het mogelijk zijn om de windmolen van op afstand
	stil te leggen en opnieuw te laten opstarten. Het ontvangen van data gebeurt via de functie UART_receive()
	waarbij de buffer receiveData[64] gevuld wordt. Eens er data ontvangen is, zal je moeten kijken wat er ontvangen
	is en je arduino laten beslissen wat hij moet doen. De commando's zullen "START" en "STOP" zijn.
	
	//--- 2.4 ---//
	Zoals reeds gezegd, zal je data moeten doorsturen via je TX en RX pinnen. Deze data moet bepaalde parameters van 
	je windmolen bevatten en moet in een specifie format doorgestuurd worden. Het format is als volgt:
	
	"RPM;Windspeed;Status;Power". Wijk niet af van dit format, anders is de data onleesbaar voor ons!
	
	- RPM: het toerental waaraan je rotor ronddraait
	- Windspeed: de gemeten windsnelheid van je anemometer, tot op 2 decimalen na de komma
	- Status: de huidige status van je windmolen, in eerste instantie ON/OFF
	- Power: het opgewekte vermogen van je windmolen tot op 2 decimalen na de komma
	
	//--- 2.5 ---//
	De functies UART_Send en UART_receive dienen om de data effectief te verzenden in samenwerking met de bijgevoegde .c en .h files.
	Hier moet je niets aan veranderen. 
	
//--- 3 ---//
code geschreven door Laurent Segers, documentatie: Jonas Verbeke.