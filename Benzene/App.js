import * as React from 'react';
import {TextInput, Platform, StyleSheet, Text, View,TouchableOpacity ,Image,Dimensions,ImageBackground} from 'react-native';
import Tts from 'react-native-tts';
import {Button, Icon} from 'native-base'
import axios from 'axios'
import LottieView from 'lottie-react-native';
import io from "socket.io-client";
window.navigator.userAgent = 'react-native';

import Voice, {
  SpeechRecognizedEvent,
  SpeechResultsEvent,
  SpeechErrorEvent,
} from '@react-native-community/voice';
import {NerManager} from 'node-nlp-rn'

export default class App extends React.Component {
  constructor(props) {
    super(props);
    Voice.onSpeechError = this.onSpeechError;
    Voice.onSpeechResults = this.onSpeechResults;
    
    this.state={
      setSettings:false,
      address:'',

    }
  }
  
  componentDidMount(){
    this.animation.play();

  }
  onSpeechError = (e) => {
    this.setState({
      error: JSON.stringify(e.error),
    });
    this.speak("Sorry I didn't understand that")
  };
  onSpeechResults =async (e ) => {
    console.log("speech results",e.value[0])
    this.socket.emit('req', e.value[0])
    this.setState({
      results: e.value,
  });

  };
  speak=(command,volume=1)=>{
    Tts.speak(command, {
      androidParams: {
        KEY_PARAM_PAN: -1,
        KEY_PARAM_VOLUME: volume,
        KEY_PARAM_STREAM: 'STREAM_MUSIC',
      },
    });
  }
  render(){

  
  return (
   <React.Fragment>
    {this.state.setSettings?<View>
      <TextInput
      style={{ height: 40, borderColor: 'gray', borderWidth: 1 }}
      onChangeText={text => this.setState({address:text})}
      value={this.state.address}
    />
    <Button transparent style={{width:'25%'}} onPress={()=>{
      this.setState({setSettings:false},()=>this.animation.play())
      
    }}>
      <Text style={{color:'#137FC9'}}>Back</Text>

      </Button>
      </View>:
    <View style={styles.container}>
    <Image resizeMode='stretch' style={{zIndex:1,width:Dimensions.get("window").width*.85,marginBottom:10,marginTop:30}} source={require('./assets/logo-t.png')} />

    <LottieView
        loop={false}
        ref={animation => {
          this.animation = animation;
        }}
        source={require('./assets/robot-anim-1.json')}
      />
      <Button disabled={false} block style={{borderRadius:10,backgroundColor:'#137FC9',position:'absolute',top:'70%',width:'90%',alignSelf:'center'}} onPress={()=>
        // Voice.start('en-US')
        // this.socket.emit('req', "hello")
        Voice.start('en-US')
        }>
        <Text style={{color:'white'}}>Command</Text>
      </Button>
      <Button block style={{borderRadius:10,backgroundColor:'#137FC9',position:'absolute',top:'80%',width:'90%',alignSelf:'center'}} onPress={()=>
      {
        this.socket = io(this.state.address,{jsonp:false,transports:['websocket'],reconnection:false});
    console.log(this.socket.connected)
    this.socket.on('connect', () => {
      this.speak("server connected successfuly")
      console.log(this.socket.connected,"duuuude"); // true
    });
    this.socket.on('connect_error',(err)=>console.log(err,err.code))
    this.socket.on("rec",msg=>{
      this.speak(msg['data'])})
      }
        }>
        <Text style={{color:'white'}}>Connect</Text>
      </Button>
      <Button block transparent style={{width:'25%',top:'90%',position:'absolute'}} onPress={()=>this.setState({setSettings:true})}>
      <Text style={{color:'#137FC9'}}>Settings</Text>

      </Button>
    </View>
    }
    </React.Fragment>
  );
}
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#fff',
    alignItems: 'center',
    justifyContent: 'flex-start',
  },
});
