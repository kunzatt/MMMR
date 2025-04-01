# --- 빌드 단계 ---
FROM node:20-alpine AS builder
WORKDIR /app

COPY . .
RUN yarn install && yarn build

# --- 실행 단계 ---
FROM node:20-alpine AS runner
WORKDIR /app

# 프로덕션 실행에 필요한 최소 파일만 복사
COPY --from=builder /app/public ./public
COPY --from=builder /app/.next ./.next
COPY --from=builder /app/node_modules ./node_modules
COPY --from=builder /app/package.json ./package.json

# 환경 변수 포트가 없으면 3000 사용
ENV PORT 3000

EXPOSE 3000
CMD ["yarn", "start"]
