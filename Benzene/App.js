import * as React from 'react';
import { Platform, StyleSheet, Text, View,TouchableOpacity ,Image,Dimensions,ImageBackground} from 'react-native';
import Tts from 'react-native-tts';
import {Button, Icon} from 'native-base'
import axios from 'axios'
import LottieView from 'lottie-react-native';
import io from "socket.io-client";

import Voice, {
  SpeechRecognizedEvent,
  SpeechResultsEvent,
  SpeechErrorEvent,
} from '@react-native-community/voice';
import {NerManager} from 'node-nlp-rn'
// import { w3cwebsocket as W3CWebSocket } from "websocket";
// const client = new W3CWebSocket('ws://192.168.43.152:8000');
export default class App extends React.Component {
  constructor(props) {
    super(props);
    Voice.onSpeechError = this.onSpeechError;
    Voice.onSpeechResults = this.onSpeechResults;
  }
  
  componentDidMount(){
    this.animation.play();
    this.socket = io("http://192.168.43.152:3000");
    this.socket.on("chat message", msg => {
          this.setState({ msg: msg
     });
    this.socket.on("send",msg=>console.log("received successful",msg))
    this.socket.emit('req', "hello")
  });
    // client.onopen = () => {
    //   console.log('WebSocket Client Connected');
    //   client.send(JSON.stringify({
    //     type: "contentchange",
    //     username: "joe",
    //     content: "text"
    //   }))

    // };
    // client.onmessage = (message) => {
    //   console.log("response received from server")
    //   console.log(message);
    // };
  }
  onSpeechError = (e) => {
    this.setState({
      error: JSON.stringify(e.error),
    });
    this.speak("Sorry I didn't understand that")
  };
  onSpeechResults =async (e ) => {
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
    <View style={styles.container}>
    <Image resizeMode='stretch' style={{zIndex:1,width:Dimensions.get("window").width*.85,marginBottom:10,marginTop:30}} source={require('./assets/logo-t.png')} />

    <LottieView
        loop={false}
        ref={animation => {
          this.animation = animation;
        }}
        source={require('./assets/robot-anim-1.json')}
      />
      <Button block style={{borderRadius:10,backgroundColor:'#137FC9',position:'absolute',top:'77%',width:'90%',alignSelf:'center'}} onPress={()=>
        // Voice.start('en-US')
        this.socket.emit('req', "hello")
        }>
        <Text style={{color:'white'}}>Command</Text>
      </Button>
    </View>
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
