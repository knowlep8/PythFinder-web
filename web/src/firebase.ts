/**
 * Firebase project setup for saved runs -- see docs/web-planner.md's step 2.8
 * revision. This config is not a secret: it names which project to talk to,
 * not a credential, and access is controlled entirely by firestore.rules
 * (open, deliberately -- see that file).
 */
import { initializeApp } from "firebase/app";
import { getFirestore } from "firebase/firestore";

const firebaseConfig = {
  projectId: "pythfinder-planner",
  appId: "1:969627824746:web:5cfc36a1f734deef109013",
  storageBucket: "pythfinder-planner.firebasestorage.app",
  apiKey: "AIzaSyAY5sTUx0Ngao_fbAPgbP9L-fjsg8NDakM",
  authDomain: "pythfinder-planner.firebaseapp.com",
  messagingSenderId: "969627824746",
};

const app = initializeApp(firebaseConfig);

export const db = getFirestore(app);
