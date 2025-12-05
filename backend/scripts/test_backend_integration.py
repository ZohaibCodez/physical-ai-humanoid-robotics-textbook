#!/usr/bin/env python3
"""
Test script to verify backend is working with Google Generative AI embeddings.
Run this to check if the backend can:
1. Generate embeddings using Google API
2. Search the vector database
3. Generate answers using Gemini
"""

import sys
import os
import asyncio
import httpx

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

async def test_embedding_service():
    """Test Google Generative AI embedding service."""
    print("\n" + "="*70)
    print("🧪 Testing Google Generative AI Embedding Service")
    print("="*70)
    
    try:
        from app.services.embeddings import google_embedding_service
        
        # Test embedding generation
        test_query = "What is ROS2?"
        print(f"\n📝 Generating embedding for: '{test_query}'")
        
        embedding = await google_embedding_service.generate_embedding(
            test_query, 
            task_type="RETRIEVAL_QUERY"
        )
        
        print(f"✅ Embedding generated successfully!")
        print(f"   - Dimension: {len(embedding)}")
        print(f"   - First 5 values: {embedding[:5]}")
        print(f"   - Model: text-embedding-004 (Google)")
        
        return True
        
    except Exception as e:
        print(f"❌ Embedding test failed: {str(e)}")
        return False


async def test_vector_search():
    """Test vector database search."""
    print("\n" + "="*70)
    print("🔍 Testing Vector Database Search")
    print("="*70)
    
    try:
        from app.services.vector_store import vector_store_service
        from app.services.embeddings import google_embedding_service
        
        # Generate query embedding
        test_query = "What are the key components of ROS2?"
        print(f"\n📝 Searching for: '{test_query}'")
        
        query_embedding = await google_embedding_service.generate_embedding(
            test_query,
            task_type="RETRIEVAL_QUERY"
        )
        
        # Search vector database
        results = await vector_store_service.search(
            query_vector=query_embedding,
            vector_name="google",
            limit=3,
            score_threshold=0.5
        )
        
        print(f"✅ Search completed successfully!")
        print(f"   - Found {len(results)} results")
        
        for i, result in enumerate(results, 1):
            print(f"\n   Result {i}:")
            print(f"   - Score: {result.get('score', 0):.3f}")
            print(f"   - Section: {result.get('metadata', {}).get('section', 'Unknown')}")
            print(f"   - Text preview: {result.get('text', '')[:100]}...")
        
        return len(results) > 0
        
    except Exception as e:
        print(f"❌ Vector search test failed: {str(e)}")
        return False


async def test_api_endpoint():
    """Test the /v1/chat/ask endpoint."""
    print("\n" + "="*70)
    print("🌐 Testing API Endpoint")
    print("="*70)
    
    try:
        api_url = "http://localhost:8000/v1/chat/ask"
        
        test_payload = {
            "session_id": "test_session_123",
            "question_text": "What is ROS2?",
            "context_mode": "full"
        }
        
        print(f"\n📤 Sending request to: {api_url}")
        print(f"   Question: {test_payload['question_text']}")
        
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(api_url, json=test_payload)
            
            if response.status_code == 200:
                data = response.json()
                print(f"✅ API request successful!")
                print(f"   - Answer length: {len(data.get('answer', ''))} characters")
                print(f"   - Citations: {len(data.get('citations', []))}")
                print(f"   - Processing time: {data.get('processing_time_ms', 0)}ms")
                print(f"\n   Answer preview:")
                print(f"   {data.get('answer', '')[:200]}...")
                return True
            else:
                print(f"❌ API request failed with status: {response.status_code}")
                print(f"   Response: {response.text}")
                return False
                
    except httpx.ConnectError:
        print(f"❌ Cannot connect to backend. Is it running?")
        print(f"   Start it with: uvicorn app.main:app --reload")
        return False
    except Exception as e:
        print(f"❌ API test failed: {str(e)}")
        return False


async def test_health_endpoint():
    """Test the /v1/health endpoint."""
    print("\n" + "="*70)
    print("💚 Testing Health Endpoint")
    print("="*70)
    
    try:
        health_url = "http://localhost:8000/v1/health"
        
        async with httpx.AsyncClient(timeout=5.0) as client:
            response = await client.get(health_url)
            
            if response.status_code == 200:
                data = response.json()
                print(f"✅ Backend is healthy!")
                print(f"   Status: {data.get('status')}")
                print(f"   Environment: {data.get('environment')}")
                return True
            else:
                print(f"❌ Health check failed: {response.status_code}")
                return False
                
    except Exception as e:
        print(f"❌ Health check failed: {str(e)}")
        return False


async def main():
    """Run all tests."""
    print("\n" + "="*70)
    print("🚀 Backend Integration Test Suite")
    print("="*70)
    print("\nTesting backend with Google Generative AI embeddings only")
    print("(No local models, cloud-only, lightweight deployment)")
    
    results = {
        "Health Check": await test_health_endpoint(),
        "Embedding Service": await test_embedding_service(),
        "Vector Search": await test_vector_search(),
        "API Endpoint": await test_api_endpoint(),
    }
    
    # Summary
    print("\n" + "="*70)
    print("📊 Test Summary")
    print("="*70)
    
    for test_name, passed in results.items():
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"   {test_name:20s} {status}")
    
    all_passed = all(results.values())
    
    print("\n" + "="*70)
    if all_passed:
        print("✅ All tests passed! Backend is working correctly.")
        print("   - Google Generative AI embeddings: ✅")
        print("   - Vector database search: ✅")
        print("   - API endpoints: ✅")
        print("\nYour backend is ready for deployment! 🚀")
    else:
        print("❌ Some tests failed. Please check the errors above.")
    print("="*70 + "\n")
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    try:
        exit_code = asyncio.run(main())
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\n⚠️  Tests interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ Fatal error: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
