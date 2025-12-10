name: Better Auth Patterns
description: Authentication with Better-Auth
Better-Auth Patterns
Server Setup
TypeScript

import { betterAuth } from 'better-auth';

export const auth = betterAuth({
  database: new Pool({ connectionString: process.env.DATABASE_URL }),
  emailAndPassword: { enabled: true },
  socialProviders: {
    github: { clientId: process.env.GITHUB_ID, clientSecret: process.env.GITHUB_SECRET }
  }
});
Client Hook
TypeScript

import { createAuthClient } from 'better-auth/react';
export const { useSession, signIn, signUp, signOut } = createAuthClient();
User Profile Schema
SQL

CREATE TABLE user_profiles (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES users(id),
  experience_level VARCHAR(20),
  interests TEXT[],
  learning_goals TEXT
);
