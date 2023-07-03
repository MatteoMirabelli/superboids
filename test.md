Ciao matteo

Se leggi questo messaggio sei nel branch che fa cose pazze ma forse rimedia qualcosa. Mi raccomando controlla spesso qui!

Problemi da risolvere:
- aggiungere assert al costruttore del predatore
- andare avanti con il multiflock: si tratta di fatto di replicare il flock, al posto dei Boid bisogna ogni volta prima chiamare 

*it.first.metodo()

in quanto si utilizza un vector di std::pair Boid + int (o unsigned int, che credo possa pure essere più funzionale anche se quando passiamo l'input dobbiamo fare una conversione in quanto il tipo intero di default è int). L'intero chiaramente serve per discriminare tra i vari sottostormi. Centro di massa, parametri e statistiche diventano chiaramente vettori.

e per i vicini ci sarebbe da implementare un 
- trova vicini x separazione: basta replicare il get neighbours sostituendo come detto sopra con .first
- trova vicini x allineamento e coesione: qui basta aggiungere il filtro su .second
- actually basta un trova vicini unico, che prende quelli degli altri stormi solo entro d_s e dello stesso stormo entro d

chiaramente da fare c'è quindi il tradurre flock in multiflock. 
Nota: non ho ereditato da flock in quanto non ha troppo senso tenersi i membri precedenti. 
Il vantaggio del vettorone è di poterli comunque ordinare tutti per essere ugualmente veloci sulla separazione. Ho scelto di usare vector + pair anzichè la map / unordered_map :
1. per non aggiungere altra carne al fuoco
2. per 

il sorting lo terrei solo per posizione indipendentemente dallo stormo di appartenenza, così si mantiene tutto efficiente.

Se ho tempo oggi pomeriggio faccio un po' io, però se riuscite ed avete voglia è meno lavoro di quanto sembri, veramente praticamente solo manovalanza.

Unici metodi nuovi chiaramente sono quelli che permettono di variare il numero di flock, e l'add boid forse è bene prenda anche l'indice di stormo cui aggiungere. Sennò si fa overload e in uno si randomizza anche lo stormo _cui_ aggiunge.
